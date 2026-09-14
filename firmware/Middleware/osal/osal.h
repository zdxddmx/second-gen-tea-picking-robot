/**
 * @file    osal.h
 * @brief   Operating-system abstraction: tasks, queues, mutexes, time.
 *
 * @details
 * # Why an OSAL and not direct FreeRTOS calls
 * The single most valuable property of this project is that **the 1 kHz control
 * loop, the force controller, the state machine and the safety monitor are
 * compiled and executed on a PC** as part of the test suite and the simulator.
 * That is only possible if those modules do not call `xTaskCreate` directly.
 *
 * The OSAL is the seam.  It has two complete backends:
 *
 * | Backend | File | Used by | Scheduling |
 * |---|---|---|---|
 * | FreeRTOS | `osal_freertos.c` | STM32F407 target | preemptive, hardware tick |
 * | Bare metal | `osal_baremetal.c` | PC tests and simulator | deterministic cooperative |
 *
 * Both implement the *same* API, so a module cannot tell which one it is
 * running on, and the tests exercise the production code path rather than a
 * re-implementation of it.
 *
 * # Task functions are STEP functions
 * This is the key API decision.  A task function passed to
 * @ref osal_task_create performs **one iteration** and returns; it must not
 * contain an infinite loop and must not block indefinitely.  The OSAL owns the
 * loop and the period:
 *
 *   - on FreeRTOS, a small wrapper task does `for(;;) { step(); wait_period(); }`;
 *   - on bare metal, the scheduler calls `step()` when the task's period elapses.
 *
 * Three things fall out of this that would otherwise be impossible:
 *
 *   1. **A test can call `control_task_step()` directly**, one iteration at a
 *      time, with a hand-driven clock.  "Run the control loop 500 times and
 *      assert the force settled" becomes a deterministic unit test with no
 *      threads, no sleeps and no flakiness.
 *   2. **The simulator is bit-reproducible.**  A cooperative scheduler that runs
 *      tasks in a fixed order at fixed simulated times produces the same trace
 *      on every machine, which is what makes the plots in `data/results/`
 *      reproducible rather than anecdotal.
 *   3. **The period is declared once**, in `config.h`, and both the OSAL and the
 *      documentation read the same macro.
 *
 * # Documented backend differences
 * The bare-metal backend cannot block: there is no other thread to switch to.
 * Any call with a non-zero timeout therefore returns immediately, and
 * @ref osal_queue_recv on an empty queue returns @ref TP_ERR_NOT_READY instead
 * of waiting.  Every task step in this project is written to be **non-blocking**
 * for exactly this reason, so the two backends behave identically in practice.
 * The difference is documented here rather than hidden, because a reader
 * comparing the two files will notice it.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_OSAL_H
#define TP_OSAL_H

#include <stdint.h>

#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Timeout value meaning "do not wait at all". */
#define OSAL_NO_WAIT 0u

/** @brief Timeout value meaning "wait indefinitely". */
#define OSAL_WAIT_FOREVER 0xFFFFFFFFu

/**
 * @brief Task priorities, low to high.
 *
 * Named levels rather than raw numbers so that the priority assignment in
 * `config.h` is reviewable as a policy ("safety sits one level below control")
 * rather than as a list of integers whose relative order has to be checked by
 * hand.  The FreeRTOS backend maps these onto `configMAX_PRIORITIES` by
 * reversing the order, with @ref OSAL_PRIO_IDLE at 0.
 */
typedef enum {
    OSAL_PRIO_IDLE = 0,     /**< Runs only when nothing else can.           */
    OSAL_PRIO_LOW = 1,      /**< Background logging.                        */
    OSAL_PRIO_BELOW_NORMAL = 2, /**< Telemetry, best-effort uploads.        */
    OSAL_PRIO_NORMAL = 3,   /**< Communication services.                    */
    OSAL_PRIO_ABOVE_NORMAL = 4, /**< Soft real-time: the state machine.     */
    OSAL_PRIO_HIGH = 5,     /**< Sensor acquisition.                        */
    OSAL_PRIO_REALTIME = 6, /**< Safety supervision.                        */
    OSAL_PRIO_CRITICAL = 7  /**< Hard real-time: the control loop.          */
} osal_prio_t;

/** @brief Opaque task handle. */
typedef struct osal_task osal_task_t;
/** @brief Opaque queue handle. */
typedef struct osal_queue osal_queue_t;
/** @brief Opaque mutex handle. */
typedef struct osal_mutex osal_mutex_t;
/** @brief Opaque counting-semaphore handle. */
typedef struct osal_sem osal_sem_t;

/**
 * @brief One iteration of a task.
 *
 * @param[in] arg  The argument supplied to @ref osal_task_create.
 *
 * @warning Must not block.  See the file header for why.
 */
typedef void (*osal_task_step_fn_t)(void *arg);

/**
 * @brief Periodic timing helper, owned by the caller.
 *
 * A plain struct rather than a hidden global so that two tasks cannot share one
 * timing state by accident, and so a test can reset the phase of one task
 * without touching the others.
 */
typedef struct {
    uint32_t last_ms;    /**< Deadline of the previous period.              */
    uint32_t period_ms;  /**< Nominal period.                               */
    uint32_t overruns;   /**< Times the deadline was missed.                */
    uint32_t executions; /**< Times @ref osal_period_wait returned.         */
} osal_period_t;

/**
 * @brief Callback invoked once per simulated tick on the bare-metal backend.
 *
 * The host platform registers this to advance the physics model, which is what
 * makes simulated time and the plant consistent: time only moves when the
 * scheduler says it does.
 */
typedef void (*osal_tick_hook_t)(uint32_t elapsed_us);

/**
 * @brief Per-task diagnostics.
 */
typedef struct {
    const char *name;        /**< Task name.                                 */
    uint32_t    executions;  /**< Iterations run.                            */
    uint32_t    overruns;    /**< Missed deadlines.                          */
    uint32_t    max_latency_us; /**< Worst observed start latency.           */
    uint32_t    period_ms;   /**< Configured period.                         */
} osal_task_stats_t;

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

/**
 * @brief Initialise the OSAL.
 *
 * On FreeRTOS this is a no-op; on bare metal it captures the start time so that
 * simulated and real millisecond counters both start at zero.
 *
 * @return @ref TP_OK, or @ref TP_ERR_IO.
 */
tp_status_t osal_init(void);

/**
 * @brief Create a periodic task.
 *
 * @param[out] out          Receives the handle; may be NULL if the caller never
 *                          needs to reference the task again.
 * @param[in]  name         Short name for logs and for the stack-watermark
 *                          report.  Not copied, so it must have static
 *                          storage duration.
 * @param[in]  step         The task body; one iteration.
 * @param[in]  arg          Argument forwarded to @p step.
 * @param[in]  prio         Priority level.
 * @param[in]  stack_words  Stack depth in 32-bit words.  Ignored by the
 *                          bare-metal backend, which uses the C stack.
 * @param[in]  period_ms    Nominal period in milliseconds.  Must be non-zero:
 *                          a task with no period cannot be scheduled
 *                          deterministically, and the FreeRTOS backend's
 *                          `vTaskDelayUntil` would divide by zero.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, @ref TP_ERR_NO_MEM, or
 *         @ref TP_ERR_OVERFLOW when the backend's task table is full.
 */
tp_status_t osal_task_create(osal_task_t **out, const char *name,
                             osal_task_step_fn_t step, void *arg,
                             osal_prio_t prio, uint32_t stack_words,
                             uint32_t period_ms);

/**
 * @brief Delete a task.
 *
 * @param[in] task  Handle, or NULL.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t osal_task_delete(osal_task_t *task);

/**
 * @brief Start the scheduler.
 *
 * On FreeRTOS this call **does not return**.  On bare metal it runs the
 * cooperative loop until @ref osal_request_stop is called, then returns, which
 * is what lets the simulator script a run of a fixed duration and then produce
 * its report.
 *
 * @return @ref TP_OK on the bare-metal backend when the loop exits.  The
 *         FreeRTOS backend is marked `noreturn` by the caller's expectation,
 *         not by the prototype, so one signature works for both.
 */
tp_status_t osal_start(void);

/**
 * @brief Ask the bare-metal scheduler to stop after the current tick.
 *
 * Ignored by the FreeRTOS backend, which has no equivalent and does not need
 * one: stopping an RTOS is a system reset.
 */
void osal_request_stop(void);

/**
 * @brief Whether a stop has been requested.
 *
 * @return Non-zero when the scheduler should exit.
 */
int osal_stop_requested(void);

/* =========================================================================
 * Time
 * ========================================================================= */

/**
 * @brief Millisecond tick.
 *
 * @return Milliseconds since @ref osal_init.
 */
uint32_t osal_time_ms(void);

/**
 * @brief Sleep for at least @p ms milliseconds.
 *
 * @param[in] ms  Duration.
 */
void osal_delay_ms(uint32_t ms);

/**
 * @brief Initialise a periodic timing helper.
 *
 * @param[in,out] p          Helper.
 * @param[in]     period_ms  Nominal period.
 */
void osal_period_init(osal_period_t *p, uint32_t period_ms);

/**
 * @brief Block (or advance simulated time) until the next period boundary.
 *
 * @param[in,out] p  Helper.
 */
void osal_period_wait(osal_period_t *p);

/**
 * @brief Number of missed deadlines since the helper was initialised.
 *
 * @param[in] p  Helper.
 * @return Overrun count.
 */
uint32_t osal_period_overruns(const osal_period_t *p);

/**
 * @brief Register the tick hook.
 *
 * @param[in] hook  Callback, or NULL to remove it.
 */
void osal_set_tick_hook(osal_tick_hook_t hook);

/* =========================================================================
 * Queues
 * ========================================================================= */

/**
 * @brief Create a fixed-capacity queue of fixed-size items.
 *
 * @param[out] out        Receives the handle.
 * @param[in]  depth      Maximum number of items.
 * @param[in]  item_size  Size of one item in bytes; must not exceed
 *                        @ref OSAL_QUEUE_MAX_ITEM.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_OVERFLOW.
 */
tp_status_t osal_queue_create(osal_queue_t **out, uint32_t depth,
                              uint32_t item_size);

/**
 * @brief Delete a queue.
 *
 * @param[in] q  Handle, or NULL.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t osal_queue_delete(osal_queue_t *q);

/**
 * @brief Enqueue an item, copying it by value.
 *
 * @param[in] q           Queue.
 * @param[in] item        Item to copy in.
 * @param[in] timeout_ms  @ref OSAL_NO_WAIT to fail immediately when full.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_OVERFLOW when full.
 */
tp_status_t osal_queue_send(osal_queue_t *q, const void *item,
                            uint32_t timeout_ms);

/**
 * @brief Dequeue an item, copying it by value.
 *
 * @param[in]  q           Queue.
 * @param[out] item        Destination.
 * @param[in]  timeout_ms  @ref OSAL_NO_WAIT to fail immediately when empty.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_NOT_READY when empty.
 */
tp_status_t osal_queue_recv(osal_queue_t *q, void *item, uint32_t timeout_ms);

/**
 * @brief Number of items currently queued.
 *
 * @param[in] q  Queue.
 * @return Item count.
 */
uint32_t osal_queue_count(const osal_queue_t *q);

/**
 * @brief Discard everything in a queue.
 *
 * @param[in] q  Queue.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t osal_queue_reset(osal_queue_t *q);

/* =========================================================================
 * Mutexes and semaphores
 * ========================================================================= */

/**
 * @brief Create a mutex implementing priority inheritance on FreeRTOS.
 *
 * @param[out] out  Receives the handle.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_NO_MEM.
 */
tp_status_t osal_mutex_create(osal_mutex_t **out);

/**
 * @brief Delete a mutex.
 *
 * @param[in] m  Handle, or NULL.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t osal_mutex_delete(osal_mutex_t *m);

/**
 * @brief Acquire a mutex.
 *
 * @param[in] m           Mutex.
 * @param[in] timeout_ms  Wait budget.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_TIMEOUT.
 */
tp_status_t osal_mutex_lock(osal_mutex_t *m, uint32_t timeout_ms);

/**
 * @brief Release a mutex.
 *
 * @param[in] m  Mutex.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t osal_mutex_unlock(osal_mutex_t *m);

/**
 * @brief Create a counting semaphore.
 *
 * @param[out] out       Receives the handle.
 * @param[in]  initial   Initial count.
 * @param[in]  max_count Maximum count.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_NO_MEM.
 */
tp_status_t osal_sem_create(osal_sem_t **out, uint32_t initial,
                            uint32_t max_count);

/**
 * @brief Delete a semaphore.
 *
 * @param[in] s  Handle, or NULL.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t osal_sem_delete(osal_sem_t *s);

/**
 * @brief Take a semaphore.
 *
 * @param[in] s           Semaphore.
 * @param[in] timeout_ms  Wait budget.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_TIMEOUT.
 */
tp_status_t osal_sem_take(osal_sem_t *s, uint32_t timeout_ms);

/**
 * @brief Give a semaphore.
 *
 * @param[in] s  Semaphore.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_OVERFLOW at the cap.
 */
tp_status_t osal_sem_give(osal_sem_t *s);

/* =========================================================================
 * Task notifications
 * ========================================================================= */

/**
 * @brief Notify a task directly, without any kernel object.
 *
 * Used for the sensor -> control handoff because it is the cheapest
 * synchronisation primitive FreeRTOS has: it writes into the target TCB rather
 * than touching a queue or a semaphore, so it is safe from an ISR and costs a
 * few instructions.  On a 1 kHz path that matters.
 *
 * @param[in] task   Target task.
 * @param[in] value  Value made available to the waiter.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t osal_task_notify_give(osal_task_t *task, uint32_t value);

/**
 * @brief Wait for a notification from the *calling* task.
 *
 * @param[in] timeout_ms     Wait budget.
 * @param[in] clear_on_exit  Non-zero to clear the notification value on return
 *                           (matching `ulTaskNotifyTake` semantics).
 * @return @ref TP_OK, or @ref TP_ERR_TIMEOUT.
 */
tp_status_t osal_task_notify_wait(uint32_t timeout_ms, int clear_on_exit);

/* =========================================================================
 * Diagnostics
 * ========================================================================= */

/**
 * @brief Copy out per-task statistics.
 *
 * @param[out] out  Destination array.
 * @param[in]  max  Capacity of @p out.
 * @return Number of entries copied.
 */
uint32_t osal_get_task_stats(osal_task_stats_t *out, uint32_t max);

/**
 * @brief Name of the backend in use, for the boot banner.
 *
 * @return `"freertos"` or `"baremetal"`.
 */
const char *osal_backend_name(void);

#ifdef __cplusplus
}
#endif

#endif /* TP_OSAL_H */
