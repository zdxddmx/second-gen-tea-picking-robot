/**
 * @file    app_tasks.h
 * @brief   Application context, the seven RTOS tasks and the command handlers.
 *
 * @details
 * # Where this file sits
 * This is the **only** file in the firmware that knows about every module at
 * once.  Everything below it knows only about its own neighbours; everything
 * above it (there is nothing above it) is the entry point.  Concentrating the
 * wiring here is what keeps the modules independently testable.
 *
 * # The seven tasks
 *
 * | Task | Rate | Priority | Responsibility | Why that rate |
 * |---|---|---|---|---|
 * | Control | 1 kHz | CRITICAL | Cascade PID, motor update | The velocity loop's bandwidth target (~8 Hz) needs at least 10x oversampling for the discrete approximation to hold; 1 kHz also matches the PWM period and gives a 1 ms worst-case reaction to a force event. |
 * | Sensor | 500 Hz | HIGH | ADC/DMA, force, encoders, limits | Twice the control rate so the control loop always has a sample that is at most 1 ms old; the force channel's 26 Hz anti-alias corner does not need more. |
 * | Comm | 100 Hz | NORMAL | UART, CAN, RS485, protocol | A 10 ms period bounds command latency at 10 ms while leaving 90 % of the CPU for control; at 921600 baud the worst-case burst is 1.2 kB, i.e. 13 ms of link time per 10 ms period, so the ring buffers absorb the difference. |
 * | Robot | 100 Hz | ABOVE_NORMAL | FSM, pick-sequence orchestration | The sequence's fastest decision is the contact handover, which the force controller already makes at 1 kHz; the FSM only needs to observe. |
 * | Safety | 100 Hz | REALTIME | All hazard checks, watchdog feed | Deliberately *slower* than control and *higher* than robot: a fault must preempt the sequence logic but must not jitter the control loop. 10 ms bounds the detection latency for every debounced check. |
 * | Telemetry | 20 Hz | BELOW_NORMAL | Periodic reports to the host | 20 Hz is what a human-readable monitor needs; 50 ms per update is well inside perception. |
 * | Debug | 5 Hz | LOW | Log flush, console, statistics | Strictly lowest: logging must never delay anything, and 200 ms of buffering is invisible to a human reading a trace. |
 *
 * # Why the sensor task exists at all
 * It would be possible to sample the ADC and the encoder directly in the
 * control task.  Separating them buys two things: the control loop's period
 * becomes independent of how long a slow conversion takes, and the sensor task
 * becomes the single writer of the shared measurement struct, so the control
 * task never reads a half-updated value 閳?a seqlock-free design that is much
 * easier to argue about than locking in a 1 kHz interrupt.
 *
 * # Why this file has no `main()`
 * `main_host.c` and `main_stm32.c` are thin: they initialise the platform, the
 * logger and the application, then hand over to the OSAL.  Both call the same
 * @ref tp_app_start, so the PC simulation and the robot run **the same
 * application**.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_APP_TASKS_H
#define TP_APP_TASKS_H

#include <stdint.h>

#include "adc_driver.h"
#include "bsp_types.h"
#include "cascade.h"
#include "can_driver.h"
#include "encoder.h"
#include "fault_manager.h"
#include "force_control.h"
#include "force_sensor.h"
#include "gripper.h"
#include "limit_switch.h"
#include "motor.h"
#include "osal.h"
#include "pid.h"
#include "protocol.h"
#include "ring_buffer.h"
#include "robot_fsm.h"
#include "rs485.h"
#include "safety_monitor.h"
#include "status.h"
#include "trajectory.h"
#include "uart_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Application kinematics and sequence constants
 * ========================================================================= */

/**
 * @brief Milliardians of joint rotation per millimetre of lateral target offset.
 *
 * The arm has a single actuated rotational joint, so the mapping from a
 * Cartesian target to a joint angle is a projection.  Over the picking
 * workspace (|y| <= 60 mm against a ~300 mm working radius) the small-angle
 * approximation `theta = y / R` is accurate to better than 2 %, i.e. 1.2 mm of
 * position error at the workspace edge.
 *
 * This is stated explicitly because it is a **limitation of the current
 * mechanism**, not a modelling convenience: a second joint would remove the
 * approximation.  See `docs/13_future_work.md`.
 */
#define TP_APP_MM_TO_MRAD 3.333f

/** @brief Joint rotation applied during the PICK phase to detach the bud, mrad. */
#define TP_APP_LIFT_MRAD 60.0f

/**
 * @brief Expected jaw gap at first contact with a nominal bud, mm.
 *
 * This is the value the flexible gripper aims to slow down before.  The vision
 * system refines it per target: see @ref tp_app_context_t::target.
 */
#define TP_APP_NOMINAL_STEM_GAP_MM 6.0f

/** @brief Confidence below which a vision target is rejected, 0..100. */
#define TP_APP_MIN_CONFIDENCE 60u

/** @brief Age after which a vision target is considered stale, ms. */
#define TP_APP_TARGET_STALE_MS 1000u

/* =========================================================================
 * Vision target
 * ========================================================================= */

/**
 * @brief A validated vision target.
 */
typedef struct {
    int16_t  x_mm;        /**< Lateral offset, millimetres.                  */
    int16_t  y_mm;        /**< Longitudinal offset, millimetres.             */
    int16_t  z_mm;        /**< Height above the datum, millimetres.          */
    uint8_t  confidence;  /**< Vision system's score, 0..100.                */
    uint16_t id;          /**< Vision system's identifier.                   */
    uint32_t rx_ms;       /**< When the robot accepted it.                   */
    int      valid;       /**< Non-zero when usable.                         */
} tp_app_target_t;

/* =========================================================================
 * Application context
 * ========================================================================= */

/**
 * @brief The whole robot, in one struct.
 *
 * A single static instance rather than a set of globals scattered across
 * modules, because:
 *   - a test can create its own instance on the stack and run the firmware
 *     against it, with no global state to reset between cases;
 *   - the memory map is one symbol, so `arm-none-eabi-nm` answers "how much RAM
 *     does the application use?" in one line;
 *   - task functions take a `void *` that is this struct, which is what makes
 *     them reentrant and therefore testable.
 */
typedef struct {
    /* --- devices ------------------------------------------------------ */
    tp_encoder_t      joint_encoder;   /**< Arm joint feedback.              */
    tp_motor_t        joint_motor;     /**< Arm joint drive.                 */
    tp_gripper_t      gripper;         /**< Gripper axis.                    */
    tp_force_sensor_t force;           /**< Load-cell channel.               */
    tp_adc_channel_t  adc_force;       /**< Force ADC (for raw diagnostics). */
    tp_adc_channel_t  adc_vbus;        /**< Supply rail monitor.             */
    tp_limit_switch_t sw_estop;        /**< E-stop, normally closed.         */
    tp_limit_switch_t sw_limit_min;    /**< Arm negative limit.              */
    tp_limit_switch_t sw_limit_max;    /**< Arm positive limit.              */
    tp_uart_link_t    link_host;       /**< Framed protocol link to the host.*/
    tp_uart_link_t    link_debug;      /**< Debug UART.                      */
    tp_rs485_t        rs485;           /**< Modbus RTU port.                 */
    tp_can_status_t   can;             /**< CAN transport.                   */

    /* --- control ------------------------------------------------------ */
    tp_cascade_t       cascade;        /**< Position/velocity cascade.       */
    tp_force_control_t grip_ctl;       /**< Flexible gripping sequence.      */
    tp_trajectory_t    lift_traj;      /**< Detachment motion during PICK.   */

    /* --- supervision -------------------------------------------------- */
    tp_fault_manager_t faults;         /**< Fault registry.                  */
    tp_safety_state_t  safety;         /**< Safety monitor state.            */
    tp_safety_inputs_t safety_in;      /**< Inputs handed to the monitor.    */
    tp_robot_fsm_t     fsm;            /**< Cycle state machine.             */

    /* --- communication ------------------------------------------------ */
    tp_protocol_t      proto;          /**< Command dispatcher.              */

    /* --- application state -------------------------------------------- */
    tp_app_target_t    target;         /**< Latest validated target.         */
    tp_robot_mode_t    mode;           /**< Operating mode.                  */

    /**
     * @brief Select position-only gripping instead of force control.
     *
     * This is the **baseline arm of Experiment 03**.  In position-only mode the
     * jaws close to a fixed gap and hold whatever force that produces, with no
     * force feedback in the loop at all —which is how the first generation of
     * this machine worked, and why it damaged buds.
     *
     * It ships as a selectable mode rather than being left out because the
     * comparison is only meaningful when both controllers run against the same
     * plant, the same trajectory generator and the same safety monitor.  The
     * safety monitor stays armed here, so the experiment also yields the honest
     * conclusion: position-only control *can* be made survivable by
     * supervision, but only by having the supervision trip; force control never
     * approaches the limit in the first place.
     *
     * Deliberately not reachable through `SET_MODE`: a host tool must not be
     * able to disable force feedback by accident.
     */
    int                grip_position_only;

    uint32_t           cycle_count;    /**< Completed cycles.                */
    uint32_t           last_comm_ms;   /**< Timestamp of the last valid frame.*/
    int                comm_seen;      /**< Non-zero once any frame arrived. */
    int                boot_ok;        /**< Self-check result.               */
    int                self_check_done;/**< Self-check has run.              */

    /* --- measurements shared with the control loop -------------------- *
     * Written only by the sensor task, read only by the control task.  On a
     * 32-bit single-core MCU a 32-bit aligned float store is atomic, so no lock
     * is required; the *struct* is not atomic, but every field is independently
     * meaningful and a one-period-old value is always acceptable for a feedback
     * signal whose own bandwidth is 8 Hz. */
    float              meas_position_mrad;
    float              meas_velocity_mrad_s;
    float              meas_force_n;
    float              meas_current_ma;
    float              meas_vbus_v;
    uint32_t           meas_tick_ms;

    /* --- timing and diagnostics --------------------------------------- */
    osal_period_t      period_control;
    osal_period_t      period_sensor;
    osal_period_t      period_comm;
    osal_period_t      period_robot;
    osal_period_t      period_safety;
    osal_period_t      period_telemetry;
    osal_period_t      period_debug;

    uint32_t           control_cycles; /**< Control iterations executed.     */
    uint32_t           tick_ms;        /**< Simulated tick counter used by
                                        *   @ref tp_app_tick_1ms.  Held in the
                                        *   context rather than in a function
                                        *   static so that a test can run two
                                        *   independent contexts, and so a
                                        *   second run starts at zero.       */
    uint32_t           sensor_cycles;  /**< Sensor iterations executed.      */
    uint32_t           comm_cycles;    /**< Communication iterations.        */
    uint32_t           robot_cycles;   /**< FSM iterations.                  */
    uint32_t           safety_cycles;  /**< Safety iterations.               */
    uint32_t           telemetry_sent; /**< Telemetry frames transmitted.    */

    /* --- fault-reporting de-duplication --------------------------------- *
     * The safety monitor re-detects an active fault every period, so the
     * "fault state" line is emitted only when the overall picture changes.
     * Without this the same line appears at 10 Hz and buries the first
     * occurrence. */
    tp_fault_code_t    last_fault_code;    /**< Previously reported code.     */
    int                last_fault_latched; /**< Previously reported latch.    */
    uint32_t           fault_log_count;    /**< Fault lines actually emitted. */
    uint32_t           deadline_misses;/**< Sum of all task overruns.        */

    int                initialised;    /**< Non-zero after tp_app_init.      */
} tp_app_context_t;

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

/**
 * @brief Initialise every device, controller and supervisor.
 *
 * Order matters and is deliberate:
 *   1. platform and logger (nothing else can report a failure before this),
 *   2. limits and E-stop, and an immediate E-stop check 閳?a robot that boots
 *      with the E-stop pressed must not energise anything, not even to self-test,
 *   3. sensors and actuators,
 *   4. controllers, supervisors and the protocol session,
 *   5. the boot self-check, which raises faults rather than returning errors.
 *
 * @param[in,out] app  Context, zeroed by the call.
 * @return @ref TP_OK when the self-check passed; the first failing step's status
 *         otherwise.  A non-@ref TP_OK return does **not** mean "do not run":
 *         the caller still starts the scheduler so that the fault can be
 *         reported over the link and cleared.
 */
tp_status_t tp_app_init(tp_app_context_t *app);

/**
 * @brief Register the seven tasks with the OSAL.
 *
 * @param[in,out] app  Context.
 * @return @ref TP_OK, or the first failing registration.
 */
tp_status_t tp_app_create_tasks(tp_app_context_t *app);

/**
 * @brief Return the application's singleton context.
 *
 * The robot is a singleton: there is one set of motors, one gripper and one
 * power supply, so the production firmware uses one static context rather than
 * threading a pointer through every call.  The functions above still take an
 * explicit pointer, which is what lets a unit test build its own context on the
 * stack and run the whole application against it with no shared state.
 *
 * @return Pointer to the process-wide context.  Never NULL.
 */
tp_app_context_t *tp_app_context(void);

/**
 * @brief Initialise the singleton context, create the tasks and start.
 *
 * This is the single symbol `main()` needs, on both targets.  It **does not
 * return** on the FreeRTOS backend; on the bare-metal backend it returns once
 * the scheduler has been asked to stop.
 *
 * @note A `TP_WEAK` definition of this function exists in
 *       `firmware/Core/app_entry.c` so that the STM32 port links and is
 *       flashable before this file is present.  The definition here is strong
 *       and therefore wins.
 */
void tp_app_start(void);

/**
 * @brief Initialise, create the tasks and start the scheduler, on a context.
 *
 * @param[in,out] app  Context.
 * @return @ref TP_OK on the bare-metal backend when the loop exits; the
 *         FreeRTOS backend does not return from this call.
 */
tp_status_t tp_app_start_ctx(tp_app_context_t *app);

/**
 * @brief Emergency shutdown: kill the drives and stop the scheduler.
 *
 * @param[in,out] app  Context, may be NULL.
 */
void tp_app_shutdown(tp_app_context_t *app);

/* =========================================================================
 * Task steps 閳?one iteration each, callable directly from a unit test
 * ========================================================================= */

/**
 * @brief One iteration of the 1 kHz control task.
 *
 * @param[in] arg  The @ref tp_app_context_t.
 */
void tp_app_control_step(void *arg);

/**
 * @brief One iteration of the 500 Hz sensor task.
 *
 * @param[in] arg  The @ref tp_app_context_t.
 */
void tp_app_sensor_step(void *arg);

/**
 * @brief One iteration of the 100 Hz communication task.
 *
 * @param[in] arg  The @ref tp_app_context_t.
 */
void tp_app_comm_step(void *arg);

/**
 * @brief One iteration of the 100 Hz robot (FSM) task.
 *
 * @param[in] arg  The @ref tp_app_context_t.
 */
void tp_app_robot_step(void *arg);

/**
 * @brief One iteration of the 100 Hz safety task.
 *
 * @param[in] arg  The @ref tp_app_context_t.
 */
void tp_app_safety_step(void *arg);

/**
 * @brief One iteration of the 20 Hz telemetry task.
 *
 * @param[in] arg  The @ref tp_app_context_t.
 */
void tp_app_telemetry_step(void *arg);

/**
 * @brief One iteration of the 5 Hz debug task.
 *
 * @param[in] arg  The @ref tp_app_context_t.
 */
void tp_app_debug_step(void *arg);

/* =========================================================================
 * Test and host entry points
 * ========================================================================= */

/**
 * @brief Run the boot self-check, raising faults rather than returning errors.
 *
 * Checks: watchdog reset cause, E-stop polarity, limit-switch plausibility,
 * encoder responsiveness, force-sensor range and noise, CAN controller state,
 * and the presence of a host link.
 *
 * @param[in,out] app  Context.
 * @return Non-zero when every check passed.
 */
int tp_app_self_check(tp_app_context_t *app);

/**
 * @brief Advance the application by one millisecond of simulated time.
 *
 * On the host, task execution is driven by the OSAL's tick hook rather than by
 * the OSAL's own scheduler when a test wants explicit control over time.  This
 * helper runs every task whose period is due, in priority order, exactly as the
 * bare-metal scheduler does 閳?so a test that calls it in a loop reproduces the
 * scheduler's behaviour byte for byte while keeping the test's own tempo.
 *
 * @param[in,out] app  Context.
 */
void tp_app_tick_1ms(tp_app_context_t *app);

/**
 * @brief One-line summary of the application's state, for logs and reports.
 *
 * @param[in]  app      Context.
 * @param[out] buffer   Destination.
 * @param[in]  capacity Capacity of @p buffer.
 * @return Characters written, excluding the terminator.
 */
uint32_t tp_app_format_status(const tp_app_context_t *app, char *buffer,
                              uint32_t capacity);

/**
 * @brief Compute the joint angle for a vision target.
 *
 * @param[in]  target         Validated target.
 * @param[out] angle_mrad     Receives the pre-grasp joint angle.
 * @param[out] approach_gap_mm Receives the expected jaw gap at contact.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_RANGE when the target is
 *         outside the reachable workspace.
 */
tp_status_t tp_app_target_to_motion(const tp_app_target_t *target,
                                    float *angle_mrad, float *approach_gap_mm);

/**
 * @brief Inject a target directly, bypassing the link.
 *
 * Used by the demo script and the tests, and by the internal vision loop of the
 * host simulator.  It performs the same validation as the protocol handler, so
 * a target that arrives this way cannot bypass a check that a target from the
 * wire would have to pass.
 *
 * @param[in,out] app         Context.
 * @param[in]     x_mm        Lateral offset.
 * @param[in]     y_mm        Longitudinal offset.
 * @param[in]     z_mm        Height.
 * @param[in]     confidence  Vision confidence, 0..100.
 * @param[in]     id          Target identifier.
 * @return @ref TP_OK, or @ref TP_ERR_RANGE when rejected.
 */
tp_status_t tp_app_inject_target(tp_app_context_t *app, int16_t x_mm, int16_t y_mm,
                                 int16_t z_mm, uint8_t confidence, uint16_t id);

/**
 * @brief Start a pick cycle programmatically.
 *
 * @param[in,out] app  Context.
 * @return @ref TP_OK, or @ref TP_ERR_STATE when the FSM or a fault refuses.
 */
tp_status_t tp_app_start_pick(tp_app_context_t *app);

/**
 * @brief Stop the cycle and open the jaws.
 *
 * @param[in,out] app  Context.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_app_stop(tp_app_context_t *app);

/**
 * @brief Reset faults and return the machine to a known state.
 *
 * @param[in,out] app  Context.
 * @return @ref TP_OK, or @ref TP_ERR_FAULT_ACTIVE when a condition persists.
 */
tp_status_t tp_app_reset(tp_app_context_t *app);

/**
 * @brief Assert the emergency stop, from software.
 *
 * @param[in,out] app  Context.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_app_emergency_stop(tp_app_context_t *app);

/**
 * @brief Select the operating mode.
 *
 * Exposed as a function as well as through the `SET_MODE` command so that the
 * host simulator, the tests and the demo script take the same code path as a
 * host tool would.
 *
 * @param[in,out] app   Context.
 * @param[in]     mode  Mode to select.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_RANGE.
 */
tp_status_t tp_app_set_mode(tp_app_context_t *app, tp_robot_mode_t mode);

/**
 * @brief Select force-controlled or position-only gripping.
 *
 * @param[in,out] app            Context.
 * @param[in]     position_only  Non-zero to use the position-only baseline.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 *
 * @warning Only legal while the robot is idle.  Switching mid-cycle would leave
 *          the jaws under one controller's command while the state machine waits
 *          for the other controller's events.
 */
tp_status_t tp_app_set_grip_mode(tp_app_context_t *app, int position_only);

#ifdef __cplusplus
}
#endif

#endif /* TP_APP_TASKS_H */
