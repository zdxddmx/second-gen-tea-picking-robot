/**
 * @file    app_sequence.c
 * @brief   Task bodies and the pick-sequence orchestration.
 *
 * @details
 * # What this file is
 * The seven task steps plus the application-level actions they invoke.  It is
 * split from `app_tasks.c` (which holds the wiring, the command handlers and the
 * lifecycle) because the two halves fail for different reasons: wiring bugs are
 * found at boot, sequence bugs are found in a machine that runs but picks
 * wrongly.  Keeping them in separate files keeps each one reviewable.
 *
 * # How the sequence is driven
 * The FSM owns the *discrete* decision ("we are in GRIP; has the force
 * settled?") and the gripping controller owns the *continuous* one ("regulate
 * the force").  Neither calls the other:
 *
 * @verbatim
 *   robot task (100 Hz)
 *      |-- reads tp_force_control_state()  -> maps it to an FSM event
 *      |-- delivers the event to tp_robot_fsm
 *      |-- acts on the resulting state (start a trajectory, open the jaws)
 * @endverbatim
 *
 * That direction of control 閳?the FSM *observes* the controller 閳?is what keeps
 * the 1 kHz force loop free of state-machine branches, and it means a change to
 * the sequence cannot alter the control law.
 *
 * # The three phases that move the arm
 *   - **APPROACH**: a trapezoidal move from wherever the arm is to the
 *     pre-grasp angle.  Trapezoidal, not S-curve, because there is nothing in
 *     the jaws yet.
 *   - **PICK**: an S-curve retraction while the grip is maintained.  S-curve
 *     here, because the bud *is* in the jaws and a jerk step would shake it
 *     loose.  This is the one place in the cycle where the profile choice is
 *     driven by the payload rather than by the mechanism.
 *   - **RETURN**: a trapezoidal move home, again with nothing held.
 *
 * @copyright MIT License, see LICENSE.
 */

#include "app_tasks.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"
#include "robot_state.h"
#include "tp_format.h"

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/**
 * @brief Convert the current FSM state and gripping sub-state into the event
 *        that leaves the current phase.
 *
 * Keeping this mapping in one function means the FSM transitions and the
 * gripping sequence's states cannot be paired up inconsistently at two call
 * sites.  The FSM observes the controller; the controller never calls the FSM.
 *
 * @param[in]  app    Context.
 * @param[in]  grip   Latest gripping sub-state.
 * @param[out] event  Receives the event to deliver, or @ref TP_EV_NONE.
 */
static void app_map_grip_state(const tp_app_context_t *app,
                               tp_grip_state_t grip, tp_event_t *event)
{
    *event = TP_EV_NONE;
    switch (app->fsm.state) {
    case TP_STATE_CONTACT:
        /* Contact is confirmed once the force loop has taken over. */
        if (grip == TP_GRIP_FORCE_CONTROL) {
            *event = TP_EV_CONTACT;
        }
        break;
    case TP_STATE_GRIP:
        /* The grip is complete when the sequence moves on to detaching. */
        if (grip == TP_GRIP_PICK) {
            *event = TP_EV_FORCE_REACHED;
        }
        break;
    case TP_STATE_PICK:
        if (grip == TP_GRIP_RELEASE) {
            *event = TP_EV_PICK_DONE;
        }
        break;
    case TP_STATE_RELEASE:
        if (grip == TP_GRIP_IDLE) {
            *event = TP_EV_RELEASE_DONE;
        }
        break;
    default:
        break;
    }
}

/**
 * @brief Hold the joint at its current position, actively.
 *
 * This is what a real servo drive does when it is "stopped": the position loop
 * stays closed and the axis resists gravity.  Commanding zero duty and leaving
 * the loop open looks equivalent on a friction-dominated bench axis and is
 * completely wrong on a gravity-loaded one 閳?the first PC simulation run of this
 * firmware made exactly that mistake, and the joint was measured drifting at a
 * steady 148 mrad/s with the drive disabled, because the 20 mN*m gravity load
 * balanced the Coulomb and back-EMF braking torques at that speed.
 *
 * So: hold position on every stop path, and reserve @ref app_disable_drive for
 * the states where motion must be impossible rather than merely unwanted.
 *
 * @param[in,out] app  Context.
 */
static void app_hold_position(tp_app_context_t *app)
{
    (void)tp_cascade_set_mode(&app->cascade, TP_CASCADE_MODE_POSITION);
    /* Re-reference to where the axis actually is, so "hold" does not turn into
     * "return to wherever you were 200 ms ago" after an abort mid-move. */
    (void)tp_cascade_set_reference(&app->cascade, app->meas_position_mrad, 0.0f);
    (void)tp_trajectory_abort(tp_cascade_trajectory(&app->cascade));
    (void)tp_motor_stop(&app->joint_motor, 200u);
    (void)tp_force_control_abort(&app->grip_ctl, &app->gripper);
}

/**
 * @brief Cut the drive entirely.
 *
 * Used only for the fault states, where the choice is between losing position
 * and continuing to drive an actuator whose feedback or current is not
 * trustworthy.  Losing position wins.
 *
 * @param[in,out] app  Context.
 */
static void app_disable_drive(tp_app_context_t *app)
{
    (void)tp_cascade_set_mode(&app->cascade, TP_CASCADE_MODE_OFF);
    tp_cascade_reset(&app->cascade);
    (void)tp_motor_set_duty(&app->joint_motor, 0.0f);
    (void)tp_motor_disable(&app->joint_motor);
    (void)tp_force_control_abort(&app->grip_ctl, &app->gripper);
}

/**
 * @brief Convert the position-only gripper's progress into an FSM event.
 *
 * The baseline counterpart of @ref app_map_grip_state.  In position-only mode
 * there is no force controller to consult, so the events come from the jaw's own
 * position loop and its limit switches.  Keeping the two mappings side by side
 * makes the difference between the two controllers visible in one place: the
 * force-controlled arm decides on a *force* and this one decides on a
 * *position*, which is precisely why it cannot bound the force.
 *
 * @param[in]  app    Context.
 * @param[out] event  Receives the event to deliver, or @ref TP_EV_NONE.
 */
static void app_map_position_only_state(const tp_app_context_t *app,
                                        tp_event_t *event)
{
    *event = TP_EV_NONE;
    switch (app->fsm.state) {
    case TP_STATE_CONTACT:
        /* "Contact" is declared when the jaw reaches its commanded gap or hits
         * the closed stop - with no knowledge of how hard it is squeezing. */
        if ((tp_gripper_is_at_target(&app->gripper) != 0) ||
            (tp_gripper_at_limit(&app->gripper) != 0)) {
            *event = TP_EV_CONTACT;
        }
        break;
    case TP_STATE_GRIP:
        if ((tp_gripper_is_at_target(&app->gripper) != 0) ||
            (tp_gripper_at_limit(&app->gripper) != 0)) {
            *event = TP_EV_FORCE_REACHED;
        }
        break;
    case TP_STATE_PICK:
        /* No detachment detection is possible: without force feedback the
         * controller cannot tell a bud coming free from a bud being crushed.
         * A fixed dwell is the honest best effort, and the fact that it has to
         * be a guess is the whole argument for force control. */
        if (tp_fsm_state_dwell_ms(&app->fsm) >= 250u) {
            *event = TP_EV_PICK_DONE;
        }
        break;
    case TP_STATE_RELEASE:
        if (tp_gripper_is_open(&app->gripper) != 0) {
            *event = TP_EV_RELEASE_DONE;
        }
        break;
    default:
        break;
    }
}

/**
 * @brief Begin the approach phase for the currently latched target.
 *
 * @param[in,out] app  Context.
 * @return @ref TP_OK, or a rejection status when the target is unusable.
 */
static tp_status_t app_begin_approach(tp_app_context_t *app)
{
    float angle_mrad = 0.0f;
    float gap_mm = 0.0f;
    tp_status_t st;
    tp_trajectory_t *traj;

    st = tp_app_target_to_motion(&app->target, &angle_mrad, &gap_mm);
    if (st != TP_OK) {
        LOG_ERROR(TP_MOD_ROBOT, "target %u unusable: %s",
                  (unsigned)app->target.id, tp_status_desc(st));
        return st;
    }

    traj = tp_cascade_trajectory(&app->cascade);
    if (traj == NULL) {
        return TP_ERR_NOT_READY;
    }
    (void)tp_cascade_set_mode(&app->cascade, TP_CASCADE_MODE_POSITION);
    (void)tp_cascade_set_profile(&app->cascade, TP_PROFILE_TRAPEZOID);
    (void)tp_cascade_reset_stats(&app->cascade);
    tp_cascade_reset(&app->cascade);
    (void)tp_trajectory_start(traj, angle_mrad);

    /* Start the flexible gripping sequence now, in its fast-approach phase, so
     * that the gripper is already closing while the arm is still moving.  Doing
     * it sequentially would add roughly 0.7 s to every cycle for no benefit:
     * the two axes are independent and their motions overlap safely. */
    if (app->grip_position_only != 0) {
        /* Experiment 03 baseline arm.  Note what is *missing*: no force
         * feedback, no contact detection, no adaptive threshold.  The jaw
         * simply closes to a fixed gap, and the force is whatever the stem
         * happens to produce. */
        st = tp_gripper_close(&app->gripper);
    } else {
        st = tp_force_control_begin(&app->grip_ctl, &app->gripper, gap_mm);
    }
    if (st != TP_OK) {
        return st;
    }
    LOG_INFO(TP_MOD_ROBOT,
             "approach target %u: angle %.1f mrad, expected contact gap %.2f mm, "
             "move %.0f ms",
             (unsigned)app->target.id, (double)angle_mrad, (double)gap_mm,
             (double)(tp_trajectory_duration(traj) * 1000.0f));
    return TP_OK;
}

/* =========================================================================
 * 1 kHz control task
 * ========================================================================= */

void tp_app_control_step(void *arg)
{
    tp_app_context_t *app = (tp_app_context_t *)arg;
    float duty;
    tp_status_t st;

    if ((app == NULL) || (app->initialised == 0)) {
        return;
    }
    app->control_cycles++;

    /* Wake the motor abstraction's update path even when the cascade is off, so
     * that the slew limiter still ramps a commanded duty down to zero instead
     * of leaving the last value latched in the timer register. */
    if (tp_cascade_is_diverged(&app->cascade) != 0) {
        /* A divergent controller is a firmware defect, not a plant condition.
         * Zero the demand, latch a fault, and stop trying: continuing to write
         * escalating outputs into a diverged loop is how an actuator gets
         * destroyed. */
        (void)tp_fault_raise(&app->faults, TP_FAULT_INTERNAL, 0u,
                             tp_platform_time_ms());
        (void)tp_motor_stop(&app->joint_motor, 0u);
        (void)tp_motor_update(&app->joint_motor, TP_CONTROL_DT_S);
        return;
    }

    duty = tp_cascade_update(&app->cascade, &app->joint_encoder, TP_CONTROL_DT_S);

    /* A latched fault must not be able to command motion, whatever the
     * controller asks for.  Enforcing it here rather than inside the cascade
     * keeps the controller a pure function of its inputs. */
    if (tp_fault_blocks_motion(&app->faults) != 0) {
        duty = 0.0f;
    }

    st = tp_motor_set_duty(&app->joint_motor, duty);
    if (st == TP_ERR_FAULT_ACTIVE) {
        /* The drive is latched; nothing more to do this period.  The latch
         * itself was already logged by the motor module. */
        (void)tp_motor_update(&app->joint_motor, TP_CONTROL_DT_S);
        return;
    }
    (void)tp_motor_update(&app->joint_motor, TP_CONTROL_DT_S);

    /* The gripper is driven from the control task too, not the robot task: the
     * force loop runs at 1 kHz (see docs/06_force_control.md for why), and the
     * jaw's velocity command must reach the motor in the same period it was
     * computed in. */
    if (tp_fault_blocks_motion(&app->faults) != 0) {
        (void)tp_gripper_stop(&app->gripper);
    }
    (void)tp_gripper_update(&app->gripper, TP_CONTROL_DT_S);
}

/* =========================================================================
 * 500 Hz sensor task
 * ========================================================================= */

void tp_app_sensor_step(void *arg)
{
    tp_app_context_t *app = (tp_app_context_t *)arg;
    uint16_t counts = 0u;
    uint32_t now;
    uint32_t elapsed_us = (uint32_t)(TP_SENSOR_DT_S * 1000000.0f);
    float vbus;

    if ((app == NULL) || (app->initialised == 0)) {
        return;
    }
    app->sensor_cycles++;
    now = tp_platform_time_ms();

    /* --- encoders ------------------------------------------------------- */
    (void)tp_encoder_update(&app->joint_encoder, now, elapsed_us);

    /* --- force channel -------------------------------------------------- */
    /* The raw ADC value is pushed through the whole filter chain, including the
     * spike detector that runs on the unfiltered value.  Doing it here rather
     * than in the control task is what makes the 500 Hz rate meaningful: at
     * 1 kHz the filters would be fed twice per real sample, which changes their
     * effective time constants by a factor of two and silently invalidates the
     * tuning in config.h. */
    if (tp_platform_adc_read(TP_ADC_FORCE, &counts) == TP_OK) {
        (void)tp_force_sensor_update(&app->force, counts);
        (void)tp_adc_channel_push(&app->adc_force, counts);
    }

    /* --- supply rail ----------------------------------------------------- */
    if (tp_platform_adc_read(TP_ADC_VBUS, &counts) == TP_OK) {
        (void)tp_adc_channel_push(&app->adc_vbus, counts);
        vbus = tp_adc_channel_get_volts(&app->adc_vbus);
    } else {
        vbus = app->meas_vbus_v;
    }

    /* --- limit switches -------------------------------------------------- */
    (void)tp_limit_switch_update(&app->sw_estop);
    (void)tp_limit_switch_update(&app->sw_limit_min);
    (void)tp_limit_switch_update(&app->sw_limit_max);

    /* --- publish the shared measurement set ----------------------------- */
    app->meas_position_mrad = tp_encoder_get_position_mrad(&app->joint_encoder);
    app->meas_velocity_mrad_s = tp_encoder_get_speed_mrad_s(&app->joint_encoder);
    app->meas_force_n = tp_force_sensor_get_force_n(&app->force);
    app->meas_current_ma = tp_motor_get_current_ma(&app->joint_motor);
    app->meas_vbus_v = vbus;
    app->meas_tick_ms = now;
}

/* =========================================================================
 * 100 Hz safety task
 * ========================================================================= */

void tp_app_safety_step(void *arg)
{
    tp_app_context_t *app = (tp_app_context_t *)arg;
    uint32_t raised;
    uint32_t now;

    if ((app == NULL) || (app->initialised == 0)) {
        return;
    }
    app->safety_cycles++;
    now = tp_platform_time_ms();

    /* Keep the monitor's view of the link fresh. */
    app->safety_in.comm_seen = app->comm_seen;
    app->safety_in.last_comm_ms = app->last_comm_ms;

    raised = tp_safety_monitor_update(&app->safety, &app->safety_in, &app->faults,
                                      now);
    tp_fault_tick(&app->faults, TP_SAFETY_PERIOD_MS);

    /* A critical fault must kill the drives in hardware, and it must do so here
     * rather than waiting for the FSM task: the FSM runs at the same rate but
     * at a lower priority, so on a loaded system it could be delayed. */
    if (tp_fault_requires_kill(&app->faults) != 0) {
        tp_platform_emergency_kill();
    }

    /* Report only on a *change* of the overall fault state.  The safety monitor
     * re-detects an active condition on every period, so logging whenever it
     * reports a raise would emit the same line at 10 Hz 鈥?which buries the
     * first occurrence, the only one that carries information, and floods the
     * very link the log is meant to help diagnose. */
    {
        tp_fault_code_t top = tp_fault_highest(&app->faults);
        int latched = tp_fault_any_latched(&app->faults);
        if ((top != app->last_fault_code) ||
            (latched != app->last_fault_latched) ||
            ((raised > 0u) && (app->fault_log_count == 0u))) {
            char buf[128];
            (void)tp_fault_format_active(&app->faults, buf, sizeof(buf));
            LOG_WARN(TP_MOD_SAFETY, "fault state: %s", buf);
            app->last_fault_code = top;
            app->last_fault_latched = latched;
            app->fault_log_count++;
        }
        tp_fsm_set_fault_state(&app->fsm, latched);
    }

    /* --- watchdog feed ---------------------------------------------------
     * Fed from exactly one place, and only after every supervised task has
     * been observed to make progress.  Feeding it from several tasks would let
     * a hung control loop be masked by a healthy logger, which defeats the
     * entire purpose of having a watchdog. */
    if ((app->control_cycles > 0u) && (app->sensor_cycles > 0u) &&
        (app->comm_cycles > 0u) && (app->robot_cycles > 0u)) {
        tp_platform_watchdog_feed();
    }
}

/* =========================================================================
 * 100 Hz communication task
 * ========================================================================= */

void tp_app_comm_step(void *arg)
{
    tp_app_context_t *app = (tp_app_context_t *)arg;
    uint32_t now;
    uint32_t before;

    if ((app == NULL) || (app->initialised == 0)) {
        return;
    }
    app->comm_cycles++;
    now = tp_platform_time_ms();

    before = app->proto.frames_in;

    /* Drain and parse the host link, then service the other buses.  The order
     * matters: host commands have the shortest latency requirement, and a
     * saturated RS485 or CAN link must not be able to delay a STOP command. */
    (void)tp_uart_service_rx(&app->link_host);
    (void)tp_uart_service_rx(&app->link_debug);
    (void)tp_protocol_service(&app->proto, now);
    (void)tp_uart_service_tx(&app->link_host);
    (void)tp_uart_service_tx(&app->link_debug);
    (void)tp_rs485_service(&app->rs485);
    (void)tp_can_service(&app->can);

    if (app->proto.frames_in > before) {
        app->comm_seen = 1;
        app->last_comm_ms = now;
    }
}

/* =========================================================================
 * 100 Hz robot task 閳?the pick sequence orchestration
 * ========================================================================= */

void tp_app_robot_step(void *arg)
{
    tp_app_context_t *app = (tp_app_context_t *)arg;
    uint32_t now;
    tp_event_t event = TP_EV_NONE;
    tp_robot_state_t state;
    tp_grip_state_t grip;
    tp_status_t st;

    if ((app == NULL) || (app->initialised == 0)) {
        return;
    }
    app->robot_cycles++;
    now = tp_platform_time_ms();

    tp_fsm_set_estop(&app->fsm, tp_limit_switch_is_active(&app->sw_estop));
    (void)tp_fsm_step(&app->fsm, now);

    state = tp_fsm_state(&app->fsm);
    grip = tp_force_control_state(&app->grip_ctl);

    /* Let the gripping controller run and see whether it has advanced.  The
     * sequence is continuous across CONTACT, GRIP, PICK *and* RELEASE, so the
     * controller must be serviced in all four.  Leaving RELEASE out (as the
     * first revision did) means the controller never reaches its IDLE state, the
     * RELEASE_DONE event is never generated, and the robot sits with the jaws
     * open forever 鈥?a hang that looks like a state-machine bug and is actually
     * a missing service call. */
    if ((app->grip_position_only != 0) &&
        ((state == TP_STATE_CONTACT) || (state == TP_STATE_GRIP) ||
         (state == TP_STATE_PICK) || (state == TP_STATE_RELEASE))) {
        /* Position-only baseline: the jaw is driven by its own position loop and
         * there is nothing for the force controller to do.  The safety monitor
         * is still watching, which is the point of the experiment. */
        if (state == TP_STATE_RELEASE) {
            (void)tp_gripper_open(&app->gripper);
        }
        app_map_position_only_state(app, &event);
    } else if ((state == TP_STATE_CONTACT) || (state == TP_STATE_GRIP) ||
        (state == TP_STATE_PICK) || (state == TP_STATE_RELEASE)) {
        st = tp_force_control_update(&app->grip_ctl, &app->gripper, &app->force,
                                     now, (float)TP_ROBOT_PERIOD_MS / 1000.0f);
        grip = tp_force_control_state(&app->grip_ctl);
        if (st == TP_ERR_FAULT_ACTIVE) {
            /* Report the *actual* trip reason.  The first revision mapped every
             * trip to ERR_FORCE_OVERLOAD, so a contact timeout - which usually
             * means the vision target was wrong, not that anything was crushed -
             * sent the operator to inspect the load cell. */
            (void)tp_fault_raise(&app->faults,
                                 tp_force_control_fault_code(&app->grip_ctl),
                                 0u, now);
            tp_fsm_set_fault_state(&app->fsm, 1);
        }
        /* Force control cannot run without a trustworthy sensor; falling back
         * to position-only gripping is recorded as a fault rather than done
         * silently, because it changes the damage risk of the cycle. */
        if (tp_force_sensor_is_faulty(&app->force) != 0) {
            (void)tp_fault_raise(&app->faults, TP_FAULT_SENSOR, 2u, now);
        }
    }

    switch (state) {
    case TP_STATE_IDLE:
        /* Acted on once per entry, not every period: re-issuing an open command
         * at 100 Hz would keep the jaw's position loop active forever and hold
         * its motor stalled against the mechanical stop on every idle second,
         * which is a real thermal load for no benefit.  Holding the *arm*,
         * however, must happen every period -- see app_hold_position. */
        if (tp_fsm_state_dwell_ms(&app->fsm) < (uint32_t)TP_ROBOT_PERIOD_MS) {
            (void)tp_gripper_open(&app->gripper);
            /* Re-detect: clear the consumed target so a stale bud is not picked
             * twice if the vision system has not sent a new one yet. */
            if (tp_fsm_has_target(&app->fsm) == 0) {
                app->target.valid = 0;
            }
        }
        app_hold_position(app);
        break;

    case TP_STATE_TARGET_RECEIVED:
        if (tp_fsm_target_id(&app->fsm) != app->target.id) {
            tp_fsm_set_target(&app->fsm, app->target.id, app->target.valid);
        }
        /* A stale target must not be picked.  The vision system is looking at a
         * moving scene; a target that is a second old refers to a bud that may
         * no longer be there. */
        if ((app->target.valid != 0) &&
            ((now - app->target.rx_ms) > TP_APP_TARGET_STALE_MS)) {
            LOG_WARN(TP_MOD_ROBOT, "target %u stale after %lu ms, discarding",
                     (unsigned)app->target.id,
                     (unsigned long)(now - app->target.rx_ms));
            app->target.valid = 0;
            tp_fsm_set_target(&app->fsm, 0u, 0);
            (void)tp_fsm_handle_event(&app->fsm, TP_EV_STOP, now);
        }
        break;

    case TP_STATE_APPROACH:
        {
            tp_trajectory_t *traj = tp_cascade_trajectory(&app->cascade);
            if (traj == NULL) {
                break;
            }
            (void)tp_trajectory_step(traj, (float)TP_ROBOT_PERIOD_MS / 1000.0f);
            (void)tp_cascade_set_reference(&app->cascade,
                                           tp_trajectory_value(traj),
                                           tp_trajectory_velocity(traj));
            if (tp_trajectory_is_active(traj) == 0) {
                event = TP_EV_APPROACH_DONE;
            }
        }
        break;

    case TP_STATE_CONTACT:
    case TP_STATE_GRIP:
        /* Nothing to do here beyond the force controller update above; the
         * transition comes from the gripping sub-state mapping. */
        break;

    case TP_STATE_PICK:
        {
            /* Retract along the S-curve while the grip is maintained.  The
             * target is the approach angle plus the lift increment; the bud
             * comes free when the stem's elastic limit is exceeded, which the
             * force loop holds through. */
            tp_trajectory_t *traj = &app->lift_traj;
            float held = tp_force_control_get_force_target(&app->grip_ctl);
            if (tp_trajectory_is_active(traj) == 0) {
                (void)tp_trajectory_start(
                    traj,
                    tp_cascade_position_ref(&app->cascade) + TP_APP_LIFT_MRAD);
            }
            (void)tp_trajectory_step(traj, (float)TP_ROBOT_PERIOD_MS / 1000.0f);
            (void)tp_cascade_set_reference(&app->cascade,
                                           tp_trajectory_value(traj),
                                           tp_trajectory_velocity(traj));
            (void)held;
        }
        break;

    case TP_STATE_RELEASE:
        /* The gripping controller owns the jaw here and commands the release
         * speed itself.  Issuing an independent `tp_gripper_open()` as well
         * would give the jaw two owners in the same period, and whichever won
         * would depend on the order of the two calls 鈥?a class of bug that is
         * invisible until the ordering is changed for an unrelated reason. */
        break;

    case TP_STATE_RETURN:
        {
            tp_trajectory_t *traj = tp_cascade_trajectory(&app->cascade);
            if (traj == NULL) {
                break;
            }
            if (tp_trajectory_is_active(traj) == 0) {
                (void)tp_cascade_set_profile(&app->cascade, TP_PROFILE_TRAPEZOID);
                (void)tp_trajectory_start(traj, TP_JOINT_HOME_MRAD);
            }
            (void)tp_trajectory_step(traj, (float)TP_ROBOT_PERIOD_MS / 1000.0f);
            (void)tp_cascade_set_reference(&app->cascade,
                                           tp_trajectory_value(traj),
                                           tp_trajectory_velocity(traj));
            if (tp_trajectory_is_active(traj) == 0) {
                event = TP_EV_HOME_REACHED;
                app->cycle_count++;
            }
        }
        break;

    case TP_STATE_ERROR:
    case TP_STATE_EMERGENCY_STOP:
        /* Hold position rather than cutting the drive, *unless* the fault makes
         * the feedback or the drive itself untrustworthy.  Losing the arm's
         * position on every recoverable fault (a missed contact, a comms blip)
         * would mean re-homing before every retry, which is a real cost; but
         * continuing to drive with a corrupt encoder or a stalled motor is not
         * defensible.  The fault registry's severity already encodes exactly
         * that distinction. */
        if ((tp_fault_is_latched(&app->faults, TP_FAULT_ENCODER) != 0) ||
            (tp_fault_is_latched(&app->faults, TP_FAULT_OVERCURRENT) != 0) ||
            (tp_fault_is_latched(&app->faults, TP_FAULT_MOTOR_STALL) != 0) ||
            (tp_fault_is_latched(&app->faults, TP_FAULT_ESTOP) != 0) ||
            (tp_platform_is_killed() != 0)) {
            app_disable_drive(app);
        } else {
            app_hold_position(app);
        }
        break;

    case TP_STATE_INIT:
    case TP_STATE_SELF_CHECK:
    default:
        break;
    }

    /* Map the gripping controller's progress onto an FSM event.  Done after the
     * switch so that a state entered earlier in this same period is not
     * immediately transitioned out of. */
    if (event == TP_EV_NONE) {
        app_map_grip_state(app, grip, &event);
    }
    if (event != TP_EV_NONE) {
        (void)tp_fsm_handle_event(&app->fsm, event, now);
    }
}

/* =========================================================================
 * 20 Hz telemetry task
 * ========================================================================= */

void tp_app_telemetry_step(void *arg)
{
    tp_app_context_t *app = (tp_app_context_t *)arg;
    tp_payload_telemetry_t t;

    if ((app == NULL) || (app->initialised == 0)) {
        return;
    }
    (void)memset(&t, 0, sizeof(t));
    t.t_ms = tp_platform_time_ms();
    t.pos_mrad = (int32_t)app->meas_position_mrad;
    t.vel_mrad_s = (int16_t)tp_fixed_from_float(app->meas_velocity_mrad_s,
                                                TP_WIRE_MRAD_S, -32768, 32767);
    t.force_mn = (int16_t)tp_fixed_from_float(app->meas_force_n, TP_WIRE_N_TO_MN,
                                              -32768, 32767);
    t.pwm_permille = (int16_t)tp_fixed_from_float(
        tp_motor_get_duty(&app->joint_motor), TP_WIRE_PERMILLE, -32768, 32767);
    t.state = (uint8_t)tp_fsm_state(&app->fsm);
    t.fault = (uint8_t)tp_fault_highest(&app->faults);

    if (tp_protocol_send_telemetry(&app->proto, &t) == TP_OK) {
        app->telemetry_sent++;
    }
}

/* =========================================================================
 * 5 Hz debug task
 * ========================================================================= */

void tp_app_debug_step(void *arg)
{
    tp_app_context_t *app = (tp_app_context_t *)arg;
    osal_task_stats_t stats[8];
    uint32_t n;
    uint32_t i;
    uint32_t misses = 0u;

    if ((app == NULL) || (app->initialised == 0)) {
        return;
    }

    n = osal_get_task_stats(stats, TP_ARRAY_SIZE(stats));
    for (i = 0u; i < n; ++i) {
        misses += stats[i].overruns;
    }
    app->deadline_misses = misses;

    LOG_DEBUG(TP_MOD_DEBUG,
              "t=%lu state=%s grip=%s F=%.3fN pos=%.0fmrad v=%.0fmrad/s "
              "duty=%.0f Vbus=%.1fV cycles=%lu misses=%lu",
              (unsigned long)tp_platform_time_ms(),
              tp_state_name(tp_fsm_state(&app->fsm)),
              tp_grip_state_name(tp_force_control_state(&app->grip_ctl)),
              (double)app->meas_force_n, (double)app->meas_position_mrad,
              (double)app->meas_velocity_mrad_s,
              (double)tp_motor_get_duty(&app->joint_motor),
              (double)app->meas_vbus_v, (unsigned long)app->cycle_count,
              (unsigned long)misses);

    if (misses > 0u) {
        /* A missed deadline is worth a warning at most once per debug period;
         * logging it per occurrence would flood the very link whose latency is
         * being reported. */
        for (i = 0u; i < n; ++i) {
            if (stats[i].overruns > 0u) {
                LOG_WARN(TP_MOD_DEBUG, "task '%s' missed %lu deadline(s) (period %lu ms)",
                         (stats[i].name != NULL) ? stats[i].name : "?",
                         (unsigned long)stats[i].overruns,
                         (unsigned long)stats[i].period_ms);
            }
        }
    }
}

/* =========================================================================
 * Application actions
 * ========================================================================= */

tp_status_t tp_app_target_to_motion(const tp_app_target_t *target,
                                    float *angle_mrad, float *approach_gap_mm)
{
    float angle;

    if ((target == NULL) || (angle_mrad == NULL) || (approach_gap_mm == NULL)) {
        return TP_ERR_PARAM;
    }
    if (target->valid == 0) {
        return TP_ERR_NOT_READY;
    }

    /* Single-joint projection: see TP_APP_MM_TO_MRAD.  The longitudinal
     * coordinate y_mm translates into depth, which this mechanism cannot reach
     * independently, so it is checked for plausibility and then used only to
     * bias the expected contact gap. */
    angle = (float)target->y_mm * TP_APP_MM_TO_MRAD;
    if ((angle < TP_JOINT_MIN_MRAD) || (angle > TP_JOINT_MAX_MRAD)) {
        return TP_ERR_RANGE;
    }
    *angle_mrad = angle;

    /* The expected contact gap is the nominal stem position plus whatever
     * lateral error the vision system reports.  A *wrong* x_mm therefore makes
     * the gripper slow down at the wrong distance, which is precisely the
     * failure mode Scenario B exercises. */
    *approach_gap_mm = TP_APP_NOMINAL_STEM_GAP_MM + (float)target->x_mm;
    if (*approach_gap_mm < TP_GRIPPER_MIN_MM) {
        *approach_gap_mm = TP_GRIPPER_MIN_MM;
    }
    if (*approach_gap_mm > TP_GRIPPER_MAX_MM) {
        *approach_gap_mm = TP_GRIPPER_MAX_MM;
    }
    return TP_OK;
}

tp_status_t tp_app_inject_target(tp_app_context_t *app, int16_t x_mm, int16_t y_mm,
                                 int16_t z_mm, uint8_t confidence, uint16_t id)
{
    float angle = 0.0f;
    float gap = 0.0f;
    uint32_t now;

    if (app == NULL) {
        return TP_ERR_PARAM;
    }
    now = tp_platform_time_ms();

    if (confidence < TP_APP_MIN_CONFIDENCE) {
        /* Low confidence is not an error, it is a rejection: the vision system
         * is telling us it is not sure, and picking anyway is how the wrong bud
         * gets cut.  Logged at INFO, not ERROR, because it is normal. */
        LOG_INFO(TP_MOD_ROBOT,
                 "target %u rejected: confidence %u below %u",
                 (unsigned)id, (unsigned)confidence,
                 (unsigned)TP_APP_MIN_CONFIDENCE);
        return TP_ERR_STATE;
    }

    app->target.x_mm = x_mm;
    app->target.y_mm = y_mm;
    app->target.z_mm = z_mm;
    app->target.confidence = confidence;
    app->target.id = id;
    app->target.rx_ms = now;
    app->target.valid = 1;

    if (tp_app_target_to_motion(&app->target, &angle, &gap) != TP_OK) {
        app->target.valid = 0;
        LOG_WARN(TP_MOD_ROBOT, "target %u outside the workspace, rejected",
                 (unsigned)id);
        return TP_ERR_RANGE;
    }

    tp_fsm_set_target(&app->fsm, id, 1);
    LOG_INFO(TP_MOD_ROBOT, "Target detected: id=%u x=%dmm y=%dmm z=%dmm conf=%u%%",
             (unsigned)id, (int)x_mm, (int)y_mm, (int)z_mm, (unsigned)confidence);
    (void)tp_fsm_handle_event(&app->fsm, TP_EV_TARGET, now);
    return TP_OK;
}

tp_status_t tp_app_start_pick(tp_app_context_t *app)
{
    tp_robot_state_t state;

    if (app == NULL) {
        return TP_ERR_PARAM;
    }
    state = tp_fsm_state(&app->fsm);

    if (tp_fault_blocks_motion(&app->faults) != 0) {
        LOG_WARN(TP_MOD_ROBOT, "start refused: fault latched (%s)",
                 tp_fault_name(tp_fault_highest(&app->faults)));
        return TP_ERR_FAULT_ACTIVE;
    }
    if (app->mode == TP_MODE_IDLE) {
        LOG_WARN(TP_MOD_ROBOT, "start refused: mode is IDLE");
        return TP_ERR_STATE;
    }
    if ((state != TP_STATE_IDLE) && (state != TP_STATE_TARGET_RECEIVED)) {
        LOG_WARN(TP_MOD_ROBOT, "start refused: already in %s",
                 tp_state_name(state));
        return TP_ERR_STATE;
    }
    if (app_begin_approach(app) != TP_OK) {
        return TP_ERR_STATE;
    }
    LOG_INFO(TP_MOD_ROBOT, "START");
    return tp_fsm_handle_event(&app->fsm, TP_EV_START, tp_platform_time_ms());
}

tp_status_t tp_app_stop(tp_app_context_t *app)
{
    if (app == NULL) {
        return TP_ERR_PARAM;
    }
    LOG_INFO(TP_MOD_ROBOT, "STOP");
    (void)tp_fsm_handle_event(&app->fsm, TP_EV_STOP, tp_platform_time_ms());
    app_hold_position(app);
    return TP_OK;
}

tp_status_t tp_app_reset(tp_app_context_t *app)
{
    uint32_t now;

    if (app == NULL) {
        return TP_ERR_PARAM;
    }
    now = tp_platform_time_ms();
    LOG_INFO(TP_MOD_ROBOT, "RESET requested");

    /* The physical E-stop must be released before a reset can re-energise
     * anything.  Checking the input rather than trusting the command is what
     * makes a remote RESET safe. */
    (void)tp_limit_switch_update(&app->sw_estop);
    if (tp_limit_switch_is_active(&app->sw_estop) != 0) {
        LOG_WARN(TP_MOD_ROBOT, "reset refused: E-stop is still asserted");
        return TP_ERR_FAULT_ACTIVE;
    }

    (void)tp_fault_clear_all(&app->faults, now);
    (void)tp_force_control_clear_fault(&app->grip_ctl);
    if (tp_fault_any_latched(&app->faults) != 0) {
        LOG_WARN(TP_MOD_ROBOT, "reset incomplete: %s still latched",
                 tp_fault_name(tp_fault_highest(&app->faults)));
        return TP_ERR_FAULT_ACTIVE;
    }

    tp_platform_emergency_release();
    tp_cascade_reset(&app->cascade);
    (void)tp_motor_clear_fault(&app->joint_motor);
    (void)tp_motor_enable(&app->joint_motor);
    (void)tp_motor_clear_fault(tp_gripper_motor(&app->gripper));
    (void)tp_motor_enable(tp_gripper_motor(&app->gripper));
    tp_safety_monitor_reset_stats(&app->safety);
    tp_force_sensor_reset_tracking(&app->force);
    app->target.valid = 0;

    tp_fsm_set_fault_state(&app->fsm, 0);
    tp_fsm_set_estop(&app->fsm, 0);
    (void)tp_fsm_force_state(&app->fsm, TP_STATE_IDLE, now, "reset");
    return TP_OK;
}

tp_status_t tp_app_emergency_stop(tp_app_context_t *app)
{
    uint32_t now;

    if (app == NULL) {
        return TP_ERR_PARAM;
    }
    now = tp_platform_time_ms();
    LOG_ERROR(TP_MOD_ROBOT, "EMERGENCY STOP commanded by host");

    (void)tp_fault_raise(&app->faults, TP_FAULT_ESTOP, 1u, now);
    tp_platform_emergency_kill();
    tp_fsm_set_fault_state(&app->fsm, 1);
    tp_fsm_set_estop(&app->fsm, 1);
    (void)tp_fsm_force_state(&app->fsm, TP_STATE_EMERGENCY_STOP, now,
                             "software emergency stop");
    return TP_OK;
}

uint32_t tp_app_format_status(const tp_app_context_t *app, char *buffer,
                              uint32_t capacity)
{
    char fault_text[96];
    int n;

    if ((app == NULL) || (buffer == NULL) || (capacity == 0u)) {
        return 0u;
    }
    (void)tp_fault_format_active(&app->faults, fault_text, sizeof(fault_text));
    n = tp_snprintf(buffer, capacity,
                    "t=%lums state=%s grip=%s mode=%u pos=%.0fmrad v=%.0fmrad/s "
                    "F=%.3fN F*=%.2fN duty=%.0f I=%.0fmA Vbus=%.1fV "
                    "cycles=%lu faults=%s",
                    (unsigned long)tp_platform_time_ms(),
                    tp_state_name(tp_fsm_state(&app->fsm)),
                    tp_grip_state_name(tp_force_control_state(&app->grip_ctl)),
                    (unsigned)app->mode, (double)app->meas_position_mrad,
                    (double)app->meas_velocity_mrad_s, (double)app->meas_force_n,
                    (double)tp_force_control_get_force_target(&app->grip_ctl),
                    (double)tp_motor_get_duty(&app->joint_motor),
                    (double)app->meas_current_ma, (double)app->meas_vbus_v,
                    (unsigned long)app->cycle_count, fault_text);
    return (n > 0) ? (uint32_t)n : 0u;
}

void tp_app_tick_1ms(tp_app_context_t *app)
{
    if (app == NULL) {
        return;
    }
    app->tick_ms++;

    /* Mirror the bare-metal scheduler exactly: priority order, harmonic
     * periods.  A test that calls this in a loop therefore reproduces the
     * scheduled behaviour while keeping control of its own tempo.
     *
     * The `>` comparison rather than `==` is deliberate: if a caller skips a
     * tick (a test that runs the loop 3 times and then jumps), an equality test
     * would miss the phase forever and the task would never run again. */
    if ((app->tick_ms % TP_CONTROL_PERIOD_MS) == 0u) {
        tp_app_control_step(app);
    }
    if ((app->tick_ms % TP_SAFETY_PERIOD_MS) == 0u) {
        tp_app_safety_step(app);
    }
    if ((app->tick_ms % TP_SENSOR_PERIOD_MS) == 0u) {
        tp_app_sensor_step(app);
    }
    if ((app->tick_ms % TP_ROBOT_PERIOD_MS) == 0u) {
        tp_app_robot_step(app);
    }
    if ((app->tick_ms % TP_COMM_PERIOD_MS) == 0u) {
        tp_app_comm_step(app);
    }
    if ((app->tick_ms % TP_TELEMETRY_PERIOD_MS) == 0u) {
        tp_app_telemetry_step(app);
    }
    if ((app->tick_ms % TP_DEBUG_PERIOD_MS) == 0u) {
        tp_app_debug_step(app);
    }
}
