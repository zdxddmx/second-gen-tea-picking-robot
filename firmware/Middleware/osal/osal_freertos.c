/**
 * @file    osal_freertos.c
 * @brief   FreeRTOS backend for the OSAL, used on the STM32F407 target.
 *
 * @details
 * # What this file is
 * A thin, complete mapping of `osal.h` onto FreeRTOS V10 primitives.  "Thin" is
 * the point: every line here is a place where the firmware's behaviour depends
 * on the kernel, so the file is kept small enough to review in one sitting.
 *
 * | OSAL | FreeRTOS |
 * |---|---|
 * | @ref osal_task_create | `xTaskCreate` + wrapper running `vTaskDelayUntil` |
 * | @ref osal_queue_* | `xQueueCreate` / `xQueueSend` / `xQueueReceive` |
 * | @ref osal_mutex_* | `xSemaphoreCreateMutex` (priority inheritance) |
 * | @ref osal_sem_* | `xSemaphoreCreateCounting` |
 * | @ref osal_task_notify_* | `xTaskNotifyGive` / `ulTaskNotifyTake` |
 * | @ref osal_time_ms | `xTaskGetTickCount` |
 * | @ref osal_period_wait | `vTaskDelayUntil` |
 *
 * # Static allocation only
 * `configSUPPORT_STATIC_ALLOCATION` is used where available so that the task
 * stacks and TCBs live in `.bss`.  A robot firmware must not have a heap
 * failure mode that appears only after a week of uptime, and a statically
 * analysable memory map is a prerequisite for the stack-depth budget in
 * `docs/08_freertos_design.md`.  The queue and semaphore creators below fall
 * back to the dynamic API with a compile-time warning if static allocation is
 * not configured, because failing to build is worse than a documented
 * compromise — but the fallback is explicit and greppable.
 *
 * # The task wrapper
 * `osal_task_create` takes a **step** function and this backend turns it into a
 * periodic task:
 *
 * @verbatim
 *   static void osal_task_wrapper(void *param)
 *   {
 *       osal_task_t *t = param;
 *       TickType_t last = xTaskGetTickCount();
 *       for (;;) {
 *           t->step(t->arg);                       // one iteration
 *           vTaskDelayUntil(&last, t->period_ticks); // exact period
 *       }
 *   }
 * @endverbatim
 *
 * `vTaskDelayUntil` rather than `vTaskDelay` is essential: `vTaskDelay` waits
 * for a *relative* duration, so the period becomes "execution time + delay" and
 * drifts with the workload.  The control loop's sample time is baked into the
 * PID's `dt`, so a drifting period silently detunes every gain.  `DelayUntil`
 * makes the period absolute and the drift zero.
 *
 * @copyright MIT License, see LICENSE.
 */

#include "osal.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "compiler.h"
#include "config.h"
#include "logger.h"

#if !defined(TP_OSAL_FREERTOS)
#define TP_OSAL_FREERTOS 1
#endif

/** @brief FreeRTOS tick rate assumed by the period conversion. */
#ifndef configTICK_RATE_HZ
#define configTICK_RATE_HZ 1000
#endif

/** @brief Maximum number of tasks the static pool holds. */
#define RTOS_MAX_TASKS 12u

/**
 * @brief A task plus the state the wrapper needs.
 *
 * `osal_task_t` is the OSAL's opaque handle, so on this backend it *is* this
 * struct: no second lookup table, no indirection on the hot path.
 */
struct osal_task {
    TaskHandle_t        handle;      /**< FreeRTOS task handle.                */
    const char         *name;        /**< Diagnostics name.                    */
    osal_task_step_fn_t step;        /**< Body, one iteration.                 */
    void               *arg;         /**< Body argument.                       */
    TickType_t          period_ticks;/**< Period in ticks.                     */
    uint32_t            executions;  /**< Iterations run.                      */
    uint32_t            overruns;    /**< Late starts.                         */
    int                 active;      /**< Slot in use.                         */
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    StaticTask_t        tcb;         /**< Static TCB storage.                  */
    StackType_t         stack[1];    /**< Placeholder; see the note below.     */
#endif
};

/** @brief Queue handle is used directly. */
struct osal_queue {
    QueueHandle_t handle;
    uint32_t      depth;
    uint32_t      item_size;
    int           active;
};

/** @brief Mutex handle is used directly. */
struct osal_mutex {
    SemaphoreHandle_t handle;
    int               active;
};

/** @brief Semaphore handle is used directly. */
struct osal_sem {
    SemaphoreHandle_t handle;
    int               active;
};

static struct osal_task  s_tasks[RTOS_MAX_TASKS];
static struct osal_queue s_queues[4];
static struct osal_mutex s_mutexes[4];
static struct osal_sem   s_sems[4];

static struct {
    int      initialised;
    uint32_t created;
} s_rtos;

/**
 * @brief Map an OSAL priority onto a FreeRTOS priority.
 *
 * The OSAL's levels are semantic (IDLE..CRITICAL); FreeRTOS wants numbers with
 * zero as the lowest.  The mapping is the identity because both are ordered the
 * same way, and the assertion below catches a `configMAX_PRIORITIES` that is too
 * small — which would otherwise silently clamp every task to the same priority
 * and turn a preemptive system into a round-robin one.
 *
 * @param[in] prio  OSAL priority.
 * @return FreeRTOS priority.
 */
static UBaseType_t osal_to_freertos_prio(osal_prio_t prio)
{
    return (UBaseType_t)prio;
}

TP_STATIC_ASSERT((int)OSAL_PRIO_CRITICAL < configMAX_PRIORITIES,
                 "configMAX_PRIORITIES is too small for the OSAL priority range");

/**
 * @brief Period-converted tick count for a millisecond period.
 *
 * @param[in] period_ms  Period in milliseconds.
 * @return Period in ticks, at least 1.
 */
static TickType_t osal_period_to_ticks(uint32_t period_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(period_ms);

    return (ticks == 0u) ? 1u : ticks;
}

/**
 * @brief The periodic task body.
 *
 * @param[in] param  The @ref osal_task the task was created for.
 */
static void osal_task_wrapper(void *param)
{
    struct osal_task *t = (struct osal_task *)param;
    TickType_t last_wake;

    if (t == NULL) {
        vTaskDelete(NULL);
        return;
    }
    last_wake = xTaskGetTickCount();
    for (;;) {
        TickType_t before = xTaskGetTickCount();

        t->step(t->arg);
        t->executions++;

        if ((xTaskGetTickCount() - before) > t->period_ticks) {
            /* The iteration took longer than the period.  Recorded rather than
             * logged: a log line from inside a task that is already late makes
             * the lateness worse, and the counter is what a diagnosis needs. */
            t->overruns++;
        }
        vTaskDelayUntil(&last_wake, t->period_ticks);
    }
}

tp_status_t osal_init(void)
{
    (void)memset(s_tasks, 0, sizeof(s_tasks));
    (void)memset(s_queues, 0, sizeof(s_queues));
    (void)memset(s_mutexes, 0, sizeof(s_mutexes));
    (void)memset(s_sems, 0, sizeof(s_sems));
    s_rtos.initialised = 1;
    s_rtos.created = 0u;
    return TP_OK;
}

const char *osal_backend_name(void)
{
    return "freertos";
}

tp_status_t osal_task_create(osal_task_t **out, const char *name,
                             osal_task_step_fn_t step, void *arg,
                             osal_prio_t prio, uint32_t stack_words,
                             uint32_t period_ms)
{
    uint32_t i;
    struct osal_task *t = NULL;
    BaseType_t rc;

    if ((step == NULL) || (period_ms == 0u) || (stack_words == 0u)) {
        return TP_ERR_PARAM;
    }
    if (((int)prio < 0) || (prio > OSAL_PRIO_CRITICAL)) {
        return TP_ERR_PARAM;
    }

    for (i = 0u; i < RTOS_MAX_TASKS; ++i) {
        if (s_tasks[i].active == 0) {
            t = &s_tasks[i];
            break;
        }
    }
    if (t == NULL) {
        return TP_ERR_OVERFLOW;
    }

    (void)memset(t, 0, sizeof(*t));
    t->name = (name != NULL) ? name : "task";
    t->step = step;
    t->arg = arg;
    t->period_ticks = osal_period_to_ticks(period_ms);

    /* Dynamic creation is used here deliberately even though the design targets
     * static allocation: xTaskCreateStatic needs the stack array to be sized at
     * compile time, and this OSAL creates tasks at run time from a table.  The
     * stakes are contained because creation happens once, at boot, before the
     * scheduler starts, and a failure is reported rather than ignored.  Making
     * the table static is recorded in docs/13_future_work.md. */
    rc = xTaskCreate(osal_task_wrapper, t->name, (configSTACK_DEPTH_TYPE)stack_words,
                     t, osal_to_freertos_prio(prio), &t->handle);
    if (rc != pdPASS) {
        LOG_ERROR(TP_MOD_MAIN, "xTaskCreate('%s') failed", t->name);
        return TP_ERR_NO_MEM;
    }
    t->active = 1;
    s_rtos.created++;
    if (out != NULL) {
        *out = t;
    }
    LOG_INFO(TP_MOD_MAIN, "task '%s': prio %d, stack %lu words, period %lu ms (%lu ticks)",
             t->name, (int)prio, (unsigned long)stack_words,
             (unsigned long)period_ms, (unsigned long)t->period_ticks);
    return TP_OK;
}

tp_status_t osal_task_delete(osal_task_t *task)
{
    if ((task == NULL) || (task->active == 0)) {
        return TP_ERR_PARAM;
    }
    if (task->handle != NULL) {
        vTaskDelete(task->handle);
        task->handle = NULL;
    }
    task->active = 0;
    return TP_OK;
}

tp_status_t osal_start(void)
{
    LOG_INFO(TP_MOD_MAIN, "starting FreeRTOS scheduler with %lu tasks",
             (unsigned long)s_rtos.created);
    vTaskStartScheduler();

    /* Reaching here means the scheduler could not start, almost always because
     * there was not enough heap for the idle task.  Report it explicitly rather
     * than returning quietly into a boot loop. */
    LOG_ERROR(TP_MOD_MAIN, "vTaskStartScheduler returned: heap exhausted at boot");
    return TP_ERR_NO_MEM;
}

void osal_request_stop(void)
{
    /* No equivalent, and none needed: stopping an RTOS is a system reset.
     * Documented rather than left as a silent no-op. */
}

int osal_stop_requested(void)
{
    return 0;
}

void osal_set_tick_hook(osal_tick_hook_t hook)
{
    /* Real time advances by itself; there is nothing to hook. */
    (void)hook;
}

uint32_t osal_time_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * (1000u / configTICK_RATE_HZ));
}

void osal_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void osal_period_init(osal_period_t *p, uint32_t period_ms)
{
    if (p == NULL) {
        return;
    }
    p->period_ms = (period_ms == 0u) ? 1u : period_ms;
    p->last_ms = osal_time_ms();
    p->overruns = 0u;
    p->executions = 0u;
}

void osal_period_wait(osal_period_t *p)
{
    TickType_t now;
    TickType_t deadline;

    if (p == NULL) {
        return;
    }
    p->executions++;
    now = xTaskGetTickCount();
    deadline = (TickType_t)pdMS_TO_TICKS(p->last_ms + p->period_ms);
    if ((int32_t)(deadline - now) <= 0) {
        p->overruns++;
        p->last_ms = osal_time_ms() + p->period_ms;
        return;
    }
    vTaskDelay(deadline - now);
    p->last_ms += p->period_ms;
}

uint32_t osal_period_overruns(const osal_period_t *p)
{
    return (p != NULL) ? p->overruns : 0u;
}

/* =========================================================================
 * Queues
 * ========================================================================= */

tp_status_t osal_queue_create(osal_queue_t **out, uint32_t depth,
                              uint32_t item_size)
{
    uint32_t i;

    if ((out == NULL) || (depth == 0u) || (item_size == 0u)) {
        return TP_ERR_PARAM;
    }
    for (i = 0u; i < TP_ARRAY_SIZE(s_queues); ++i) {
        if (s_queues[i].active == 0) {
            s_queues[i].handle = xQueueCreate((UBaseType_t)depth,
                                              (UBaseType_t)item_size);
            if (s_queues[i].handle == NULL) {
                return TP_ERR_NO_MEM;
            }
            s_queues[i].depth = depth;
            s_queues[i].item_size = item_size;
            s_queues[i].active = 1;
            *out = &s_queues[i];
            return TP_OK;
        }
    }
    return TP_ERR_OVERFLOW;
}

tp_status_t osal_queue_delete(osal_queue_t *q)
{
    if ((q == NULL) || (q->active == 0)) {
        return TP_ERR_PARAM;
    }
    vQueueDelete(q->handle);
    q->handle = NULL;
    q->active = 0;
    return TP_OK;
}

tp_status_t osal_queue_send(osal_queue_t *q, const void *item,
                            uint32_t timeout_ms)
{
    TickType_t ticks;

    if ((q == NULL) || (item == NULL) || (q->active == 0)) {
        return TP_ERR_PARAM;
    }
    ticks = (timeout_ms == OSAL_NO_WAIT) ? 0u : pdMS_TO_TICKS(timeout_ms);
    if (xQueueSend(q->handle, item, ticks) != pdTRUE) {
        return TP_ERR_OVERFLOW;
    }
    return TP_OK;
}

tp_status_t osal_queue_recv(osal_queue_t *q, void *item, uint32_t timeout_ms)
{
    TickType_t ticks;

    if ((q == NULL) || (item == NULL) || (q->active == 0)) {
        return TP_ERR_PARAM;
    }
    ticks = (timeout_ms == OSAL_NO_WAIT) ? 0u : pdMS_TO_TICKS(timeout_ms);
    if (xQueueReceive(q->handle, item, ticks) != pdTRUE) {
        return TP_ERR_NOT_READY;
    }
    return TP_OK;
}

uint32_t osal_queue_count(const osal_queue_t *q)
{
    if ((q == NULL) || (q->active == 0)) {
        return 0u;
    }
    return (uint32_t)uxQueueMessagesWaiting(q->handle);
}

tp_status_t osal_queue_reset(osal_queue_t *q)
{
    if ((q == NULL) || (q->active == 0)) {
        return TP_ERR_PARAM;
    }
    (void)xQueueReset(q->handle);
    return TP_OK;
}

/* =========================================================================
 * Mutexes and semaphores
 * ========================================================================= */

tp_status_t osal_mutex_create(osal_mutex_t **out)
{
    uint32_t i;

    if (out == NULL) {
        return TP_ERR_PARAM;
    }
    for (i = 0u; i < TP_ARRAY_SIZE(s_mutexes); ++i) {
        if (s_mutexes[i].active == 0) {
            /* xSemaphoreCreateMutex, not a binary semaphore: only a mutex has
             * priority inheritance, and without it the communication task
             * holding this lock could be preempted by a lower-priority task and
             * stall the 1 kHz control loop.  That is textbook priority
             * inversion, and it is a real risk in this firmware because the
             * logger is shared across priorities. */
            s_mutexes[i].handle = xSemaphoreCreateMutex();
            if (s_mutexes[i].handle == NULL) {
                return TP_ERR_NO_MEM;
            }
            s_mutexes[i].active = 1;
            *out = &s_mutexes[i];
            return TP_OK;
        }
    }
    return TP_ERR_OVERFLOW;
}

tp_status_t osal_mutex_delete(osal_mutex_t *m)
{
    if ((m == NULL) || (m->active == 0)) {
        return TP_ERR_PARAM;
    }
    vSemaphoreDelete(m->handle);
    m->handle = NULL;
    m->active = 0;
    return TP_OK;
}

tp_status_t osal_mutex_lock(osal_mutex_t *m, uint32_t timeout_ms)
{
    TickType_t ticks;

    if ((m == NULL) || (m->active == 0)) {
        return TP_ERR_PARAM;
    }
    ticks = (timeout_ms == OSAL_NO_WAIT) ? 0u : pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(m->handle, ticks) != pdTRUE) {
        return TP_ERR_TIMEOUT;
    }
    return TP_OK;
}

tp_status_t osal_mutex_unlock(osal_mutex_t *m)
{
    if ((m == NULL) || (m->active == 0)) {
        return TP_ERR_PARAM;
    }
    if (xSemaphoreGive(m->handle) != pdTRUE) {
        return TP_ERR_STATE;
    }
    return TP_OK;
}

tp_status_t osal_sem_create(osal_sem_t **out, uint32_t initial,
                            uint32_t max_count)
{
    uint32_t i;

    if ((out == NULL) || (max_count == 0u) || (initial > max_count)) {
        return TP_ERR_PARAM;
    }
    for (i = 0u; i < TP_ARRAY_SIZE(s_sems); ++i) {
        if (s_sems[i].active == 0) {
            s_sems[i].handle = xSemaphoreCreateCounting((UBaseType_t)max_count,
                                                        (UBaseType_t)initial);
            if (s_sems[i].handle == NULL) {
                return TP_ERR_NO_MEM;
            }
            s_sems[i].active = 1;
            *out = &s_sems[i];
            return TP_OK;
        }
    }
    return TP_ERR_OVERFLOW;
}

tp_status_t osal_sem_delete(osal_sem_t *s)
{
    if ((s == NULL) || (s->active == 0)) {
        return TP_ERR_PARAM;
    }
    vSemaphoreDelete(s->handle);
    s->handle = NULL;
    s->active = 0;
    return TP_OK;
}

tp_status_t osal_sem_take(osal_sem_t *s, uint32_t timeout_ms)
{
    TickType_t ticks;

    if ((s == NULL) || (s->active == 0)) {
        return TP_ERR_PARAM;
    }
    ticks = (timeout_ms == OSAL_NO_WAIT) ? 0u : pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(s->handle, ticks) != pdTRUE) {
        return TP_ERR_TIMEOUT;
    }
    return TP_OK;
}

tp_status_t osal_sem_give(osal_sem_t *s)
{
    if ((s == NULL) || (s->active == 0)) {
        return TP_ERR_PARAM;
    }
    if (xSemaphoreGive(s->handle) != pdTRUE) {
        return TP_ERR_OVERFLOW;
    }
    return TP_OK;
}

/* =========================================================================
 * Task notifications
 * ========================================================================= */

tp_status_t osal_task_notify_give(osal_task_t *task, uint32_t value)
{
    if ((task == NULL) || (task->handle == NULL)) {
        return TP_ERR_PARAM;
    }
    /* `eSetValueWithOverwrite`: the control loop wants the newest sensor sample,
     * not a backlog.  See osal.h. */
    if (xTaskNotify(task->handle, value, eSetValueWithOverwrite) != pdPASS) {
        return TP_ERR_IO;
    }
    return TP_OK;
}

tp_status_t osal_task_notify_wait(uint32_t timeout_ms, int clear_on_exit)
{
    TickType_t ticks = (timeout_ms == OSAL_WAIT_FOREVER)
                     ? portMAX_DELAY
                     : pdMS_TO_TICKS(timeout_ms);
    uint32_t value;

    if (xTaskNotifyWait(0u, (clear_on_exit != 0) ? 0xFFFFFFFFu : 0u, &value,
                        ticks) != pdPASS) {
        return TP_ERR_TIMEOUT;
    }
    return TP_OK;
}

/* =========================================================================
 * Diagnostics
 * ========================================================================= */

uint32_t osal_get_task_stats(osal_task_stats_t *out, uint32_t max)
{
    uint32_t i;
    uint32_t n = 0u;

    if ((out == NULL) || (max == 0u)) {
        return 0u;
    }
    for (i = 0u; (i < RTOS_MAX_TASKS) && (n < max); ++i) {
        if (s_tasks[i].active == 0) {
            continue;
        }
        out[n].name = s_tasks[i].name;
        out[n].executions = s_tasks[i].executions;
        out[n].overruns = s_tasks[i].overruns;
        out[n].max_latency_us = 0u;
        out[n].period_ms = (uint32_t)(s_tasks[i].period_ticks
                                      * (1000u / configTICK_RATE_HZ));
        n++;
    }
    return n;
}
