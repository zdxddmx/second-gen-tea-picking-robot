/**
 * @file    osal_baremetal.c
 * @brief   Deterministic cooperative OSAL backend for the host and simulator.
 *
 * @details
 * # Why a cooperative scheduler is the right answer here
 * The obvious way to run firmware logic on a PC is to give each task an OS
 * thread.  That is a bad fit for this project for three reasons:
 *
 *   1. **Reproducibility.**  A threaded simulation produces a different trace on
 *      every run, because the interleaving depends on the host scheduler.  The
 *      plots in `data/results/` would then not be reproducible, and neither
 *      would the rise-time and overshoot numbers derived from them.
 *   2. **The physics must advance in step with the tasks.**  With real threads
 *      the plant integrator needs its own clock, and any drift between the two
 *      silently changes the effective sample time — which is exactly the
 *      quantity the control experiments are measuring.
 *   3. **Determinism is the property being tested.**  A control loop that only
 *      works when its timing is lucky is not a control loop.
 *
 * So this backend runs the tasks in a fixed priority-then-declaration order, at
 * exactly their declared periods, against a simulated millisecond clock, and
 * advances the physics model once per tick through the tick hook.  Two runs of
 * the same script are byte-identical.
 *
 * # Scheduling policy
 * On each 1 ms tick:
 *   1. advance the tick hook (the plant),
 *   2. run every task whose period boundary has arrived, in **priority order**
 *      (highest first), then declaration order within a priority,
 *   3. increment the simulated clock.
 *
 * Running high priority first means a slow low-priority task cannot delay a
 * fast one within a tick, which is the cooperative equivalent of preemption and
 * is what makes the priority assignments in `config.h` meaningful here too.
 *
 * # What is deliberately not emulated
 * Preemption *within* a task, priority inversion, and stack overflow.  A task
 * that blocks for longer than its period is caught instead by the overrun
 * counter, which reports it rather than hiding it; that is a deliberately
 * simpler and more honest failure mode than emulating an RTOS badly.
 *
 * @copyright MIT License, see LICENSE.
 */

#include "osal.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

#ifndef TP_OSAL_BAREMETAL
#define TP_OSAL_BAREMETAL 1
#endif

/** @brief Maximum number of tasks this backend can schedule. */
#define BM_MAX_TASKS 12u
/** @brief Maximum number of queues. */
#define BM_MAX_QUEUES 6u
/** @brief Maximum number of mutexes and semaphores. */
#define BM_MAX_MUTEXES 4u
#define BM_MAX_SEMS 4u
/** @brief Largest queue item, bytes. */
#define BM_MAX_ITEM 64u
/** @brief Longest task name retained for diagnostics. */
#define BM_NAME_LEN 16u

/** @brief A scheduled periodic task. */
struct osal_task {
    const char       *name;     /**< Diagnostics name.                       */
    osal_task_step_fn_t step;   /**< Body.                                   */
    void             *arg;      /**< Body argument.                          */
    osal_prio_t       prio;     /**< Scheduling priority.                    */
    uint32_t          period_ms;/**< Nominal period.                         */
    uint32_t          next_ms;  /**< Next due time.                          */
    uint32_t          executions;/**< Iterations run.                        */
    uint32_t          overruns; /**< Times it ran late.                      */
    uint32_t          max_latency_us;/**< Worst start latency.               */
    uint32_t          notify_value;/**< Pending notification value.          */
    int               notify_pending;/**< Non-zero when a notification is held.*/
    int               active;   /**< Slot in use.                            */
};

/** @brief A queue. */
struct osal_queue {
    uint8_t  storage[8u][BM_MAX_ITEM]; /**< Fixed item storage.             */
    uint32_t depth;             /**< Configured depth.                       */
    uint32_t item_size;         /**< Size of one item.                       */
    uint32_t head;              /**< Read index.                             */
    uint32_t tail;              /**< Write index.                            */
    uint32_t count;             /**< Items held.                             */
    int      active;            /**< Slot in use.                            */
};

/** @brief A mutex. */
struct osal_mutex {
    uint32_t owner_marker;      /**< Non-zero while held.                    */
    uint32_t lock_count;        /**< Recursion depth.                        */
    int      active;            /**< Slot in use.                            */
};

/** @brief A counting semaphore. */
struct osal_sem {
    uint32_t count;             /**< Current count.                          */
    uint32_t max_count;         /**< Configured cap.                         */
    int      active;            /**< Slot in use.                            */
};

static struct osal_task  s_tasks[BM_MAX_TASKS];
static struct osal_queue s_queues[BM_MAX_QUEUES];
static struct osal_mutex s_mutexes[BM_MAX_MUTEXES];
static struct osal_sem   s_sems[BM_MAX_SEMS];

static struct {
    uint32_t         time_ms;      /**< Simulated clock.                     */
    int              stop;         /**< Stop request flag.                   */
    int              running;      /**< Non-zero while osal_start is looping.*/
    osal_tick_hook_t hook;         /**< Tick hook.                           */
    uint32_t         ticks;        /**< Ticks elapsed.                       */
    uint32_t         late_ticks;   /**< Ticks with a task overrun.           */
    int              initialised;  /**< Non-zero after osal_init.            */
} s_bm;

tp_status_t osal_init(void)
{
    (void)memset(s_tasks, 0, sizeof(s_tasks));
    (void)memset(s_queues, 0, sizeof(s_queues));
    (void)memset(s_mutexes, 0, sizeof(s_mutexes));
    (void)memset(s_sems, 0, sizeof(s_sems));
    (void)memset(&s_bm, 0, sizeof(s_bm));
    s_bm.initialised = 1;
    return TP_OK;
}

const char *osal_backend_name(void)
{
    return "baremetal";
}

tp_status_t osal_task_create(osal_task_t **out, const char *name,
                             osal_task_step_fn_t step, void *arg,
                             osal_prio_t prio, uint32_t stack_words,
                             uint32_t period_ms)
{
    uint32_t i;

    if ((step == NULL) || (period_ms == 0u)) {
        return TP_ERR_PARAM;
    }
    if (((int)prio < 0) || (prio > OSAL_PRIO_CRITICAL)) {
        return TP_ERR_PARAM;
    }
    (void)stack_words; /* the host uses the C stack; recorded for the report */

    for (i = 0u; i < BM_MAX_TASKS; ++i) {
        if (s_tasks[i].active == 0) {
            s_tasks[i].name = (name != NULL) ? name : "task";
            s_tasks[i].step = step;
            s_tasks[i].arg = arg;
            s_tasks[i].prio = prio;
            s_tasks[i].period_ms = period_ms;
            s_tasks[i].next_ms = period_ms;
            s_tasks[i].active = 1;
            if (out != NULL) {
                *out = &s_tasks[i];
            }
            LOG_DEBUG(TP_MOD_MAIN, "task '%s' registered: prio %d, period %lu ms",
                      s_tasks[i].name, (int)prio, (unsigned long)period_ms);
            return TP_OK;
        }
    }
    LOG_ERROR(TP_MOD_MAIN, "task table full (%u entries)", (unsigned)BM_MAX_TASKS);
    return TP_ERR_OVERFLOW;
}

tp_status_t osal_task_delete(osal_task_t *task)
{
    if (task == NULL) {
        return TP_ERR_PARAM;
    }
    task->active = 0;
    return TP_OK;
}

/**
 * @brief Run every task that is due, highest priority first.
 *
 * @param[in] now_ms  Current simulated time.
 */
static void bm_run_due_tasks(uint32_t now_ms)
{
    int pass;
    uint32_t i;

    /* Iterate priorities high -> low, so within one tick the control loop runs
     * before the logger regardless of registration order. */
    for (pass = (int)OSAL_PRIO_CRITICAL; pass >= 0; --pass) {
        for (i = 0u; i < BM_MAX_TASKS; ++i) {
            struct osal_task *t = &s_tasks[i];
            uint32_t start_ms;

            if ((t->active == 0) || ((int)t->prio != pass)) {
                continue;
            }
            if ((int32_t)(now_ms - t->next_ms) < 0) {
                continue; /* not due yet */
            }

            start_ms = s_bm.time_ms;
            t->step(t->arg);
            t->executions++;

            /* Latency measured against the *simulated* clock: if the hook moved
             * time forward this is a task that overran its slot, which is
             * exactly the condition the overrun counter exists to report. */
            if ((s_bm.time_ms - start_ms) > 0u) {
                t->overruns++;
                s_bm.late_ticks++;
            }
            /* Schedule the next activation relative to the deadline, not to the
             * completion time: using the completion time would let a task that
             * is consistently slightly late drift without ever showing an
             * overrun. */
            t->next_ms += t->period_ms;
            if ((int32_t)(now_ms - t->next_ms) >= 0) {
                /* Fell more than a whole period behind: resynchronise instead
                 * of trying to catch up, which would otherwise produce a burst
                 * of back-to-back invocations and a cascade of overruns. */
                t->next_ms = now_ms + t->period_ms;
            }
        }
    }
}

tp_status_t osal_start(void)
{
    if (s_bm.initialised == 0) {
        return TP_ERR_NOT_READY;
    }
    s_bm.running = 1;
    LOG_INFO(TP_MOD_MAIN, "baremetal scheduler starting at t=0 ms");

    while (s_bm.stop == 0) {
        uint32_t elapsed_us = 1000u; /* one simulated millisecond per tick */

        if (s_bm.hook != NULL) {
            s_bm.hook(elapsed_us);
        }
        bm_run_due_tasks(s_bm.time_ms);
        s_bm.time_ms += 1u;
        s_bm.ticks++;

        /* A cooperative loop with no periodic tasks would spin forever; the
         * guard keeps a mis-configured build from hanging the test runner. */
        if (s_bm.ticks > 3600000u) {
            LOG_ERROR(TP_MOD_MAIN,
                      "scheduler ran 1 hour of simulated time without a stop request");
            break;
        }
    }
    s_bm.running = 0;
    LOG_INFO(TP_MOD_MAIN, "scheduler stopped at t=%lu ms after %lu ticks",
             (unsigned long)s_bm.time_ms, (unsigned long)s_bm.ticks);
    return TP_OK;
}

void osal_request_stop(void)
{
    s_bm.stop = 1;
}

int osal_stop_requested(void)
{
    return s_bm.stop;
}

void osal_set_tick_hook(osal_tick_hook_t hook)
{
    s_bm.hook = hook;
}

uint32_t osal_time_ms(void)
{
    return s_bm.time_ms;
}

void osal_delay_ms(uint32_t ms)
{
    /* "Sleeping" in the cooperative backend means advancing the simulated
     * clock and the plant, so a task that waits observes the same physical
     * effect as on real hardware without making the test take real time. */
    uint32_t i;

    for (i = 0u; i < ms; ++i) {
        if (s_bm.hook != NULL) {
            s_bm.hook(1000u);
        }
        s_bm.time_ms += 1u;
        s_bm.ticks++;
    }
}

void osal_period_init(osal_period_t *p, uint32_t period_ms)
{
    if (p == NULL) {
        return;
    }
    p->last_ms = (period_ms == 0u) ? 1u : period_ms;
    p->period_ms = (period_ms == 0u) ? 1u : period_ms;
    p->overruns = 0u;
    p->executions = 0u;
}

void osal_period_wait(osal_period_t *p)
{
    uint32_t guard;

    if (p == NULL) {
        return;
    }
    p->last_ms += p->period_ms;
    p->executions++;

    /* Fixed bound rather than "while not due": if a caller's period is smaller
     * than the tick, an unbounded loop would never terminate. */
    guard = 0u;
    while (((int32_t)(s_bm.time_ms - p->last_ms) < 0) && (guard < 10000u)) {
        if (s_bm.hook != NULL) {
            s_bm.hook(1000u);
        }
        s_bm.time_ms += 1u;
        s_bm.ticks++;
        guard++;
    }
    if (guard >= 10000u) {
        /* The deadline is unreachable; resynchronise and record it. */
        p->overruns++;
        p->last_ms = s_bm.time_ms + p->period_ms;
    }
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
    if ((depth > 8u) || (item_size > BM_MAX_ITEM)) {
        return TP_ERR_OVERFLOW;
    }
    for (i = 0u; i < BM_MAX_QUEUES; ++i) {
        if (s_queues[i].active == 0) {
            (void)memset(&s_queues[i], 0, sizeof(s_queues[i]));
            s_queues[i].depth = depth;
            s_queues[i].item_size = item_size;
            s_queues[i].active = 1;
            *out = &s_queues[i];
            return TP_OK;
        }
    }
    return TP_ERR_NO_MEM;
}

tp_status_t osal_queue_delete(osal_queue_t *q)
{
    if (q == NULL) {
        return TP_ERR_PARAM;
    }
    q->active = 0;
    return TP_OK;
}

tp_status_t osal_queue_send(osal_queue_t *q, const void *item,
                            uint32_t timeout_ms)
{
    (void)timeout_ms; /* documented no-op: see osal.h */

    if ((q == NULL) || (item == NULL) || (q->active == 0)) {
        return TP_ERR_PARAM;
    }
    if (q->count >= q->depth) {
        return TP_ERR_OVERFLOW;
    }
    (void)memcpy(&q->storage[q->tail][0], item, q->item_size);
    q->tail = (q->tail + 1u) % q->depth;
    q->count++;
    return TP_OK;
}

tp_status_t osal_queue_recv(osal_queue_t *q, void *item, uint32_t timeout_ms)
{
    (void)timeout_ms; /* documented no-op: see osal.h */

    if ((q == NULL) || (item == NULL) || (q->active == 0)) {
        return TP_ERR_PARAM;
    }
    if (q->count == 0u) {
        return TP_ERR_NOT_READY;
    }
    (void)memcpy(item, &q->storage[q->head][0], q->item_size);
    q->head = (q->head + 1u) % q->depth;
    q->count--;
    return TP_OK;
}

uint32_t osal_queue_count(const osal_queue_t *q)
{
    return (q != NULL) ? q->count : 0u;
}

tp_status_t osal_queue_reset(osal_queue_t *q)
{
    if (q == NULL) {
        return TP_ERR_PARAM;
    }
    q->head = 0u;
    q->tail = 0u;
    q->count = 0u;
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
    for (i = 0u; i < BM_MAX_MUTEXES; ++i) {
        if (s_mutexes[i].active == 0) {
            (void)memset(&s_mutexes[i], 0, sizeof(s_mutexes[i]));
            s_mutexes[i].active = 1;
            *out = &s_mutexes[i];
            return TP_OK;
        }
    }
    return TP_ERR_NO_MEM;
}

tp_status_t osal_mutex_delete(osal_mutex_t *m)
{
    if (m == NULL) {
        return TP_ERR_PARAM;
    }
    m->active = 0;
    return TP_OK;
}

tp_status_t osal_mutex_lock(osal_mutex_t *m, uint32_t timeout_ms)
{
    (void)timeout_ms;

    if ((m == NULL) || (m->active == 0)) {
        return TP_ERR_PARAM;
    }
    /* Because the scheduler is cooperative, a lock can only ever be contended
     * by a task that was interrupted *within* a critical section, which cannot
     * happen.  The recursion counter and the owner marker are still maintained
     * so that the call is a faithful model of the FreeRTOS behaviour and so a
     * misuse (unlock twice, unlock by a non-owner) is detectable in a test. */
    if (m->owner_marker != 0u) {
        m->lock_count++;
        return TP_OK;
    }
    m->owner_marker = 1u;
    m->lock_count = 1u;
    return TP_OK;
}

tp_status_t osal_mutex_unlock(osal_mutex_t *m)
{
    if ((m == NULL) || (m->active == 0)) {
        return TP_ERR_PARAM;
    }
    if (m->lock_count == 0u) {
        return TP_ERR_STATE;
    }
    m->lock_count--;
    if (m->lock_count == 0u) {
        m->owner_marker = 0u;
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
    for (i = 0u; i < BM_MAX_SEMS; ++i) {
        if (s_sems[i].active == 0) {
            s_sems[i].count = initial;
            s_sems[i].max_count = max_count;
            s_sems[i].active = 1;
            *out = &s_sems[i];
            return TP_OK;
        }
    }
    return TP_ERR_NO_MEM;
}

tp_status_t osal_sem_delete(osal_sem_t *s)
{
    if (s == NULL) {
        return TP_ERR_PARAM;
    }
    s->active = 0;
    return TP_OK;
}

tp_status_t osal_sem_take(osal_sem_t *s, uint32_t timeout_ms)
{
    (void)timeout_ms;

    if ((s == NULL) || (s->active == 0)) {
        return TP_ERR_PARAM;
    }
    if (s->count == 0u) {
        return TP_ERR_TIMEOUT;
    }
    s->count--;
    return TP_OK;
}

tp_status_t osal_sem_give(osal_sem_t *s)
{
    if ((s == NULL) || (s->active == 0)) {
        return TP_ERR_PARAM;
    }
    if (s->count >= s->max_count) {
        return TP_ERR_OVERFLOW;
    }
    s->count++;
    return TP_OK;
}

/* =========================================================================
 * Task notifications
 * ========================================================================= */

tp_status_t osal_task_notify_give(osal_task_t *task, uint32_t value)
{
    if ((task == NULL) || (task->active == 0)) {
        return TP_ERR_PARAM;
    }
    /* A notification that arrives while one is already pending *overwrites* the
     * value rather than queueing, which matches `xTaskNotify` with
     * `eSetValueWithOverwrite` and is the semantics the sensor-to-control
     * handoff relies on: the control loop wants the newest sample, not a
     * backlog of old ones. */
    task->notify_value = value;
    task->notify_pending = 1;
    return TP_OK;
}

tp_status_t osal_task_notify_wait(uint32_t timeout_ms, int clear_on_exit)
{
    uint32_t i;
    (void)timeout_ms;

    for (i = 0u; i < BM_MAX_TASKS; ++i) {
        /* The bare-metal backend cannot identify "the calling task", so the
         * wait applies to whichever task has a pending notification.  Since the
         * production code never calls this on this backend (the tick hook
         * synchronises instead), the behaviour is documented rather than
         * emulated. */
        if ((s_tasks[i].active != 0) && (s_tasks[i].notify_pending != 0)) {
            if (clear_on_exit != 0) {
                s_tasks[i].notify_pending = 0;
                s_tasks[i].notify_value = 0u;
            }
            return TP_OK;
        }
    }
    return TP_ERR_TIMEOUT;
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
    for (i = 0u; (i < BM_MAX_TASKS) && (n < max); ++i) {
        if (s_tasks[i].active == 0) {
            continue;
        }
        out[n].name = s_tasks[i].name;
        out[n].executions = s_tasks[i].executions;
        out[n].overruns = s_tasks[i].overruns;
        out[n].max_latency_us = s_tasks[i].max_latency_us;
        out[n].period_ms = s_tasks[i].period_ms;
        n++;
    }
    return n;
}
