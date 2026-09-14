/**
 * @file    safety_monitor.c
 * @brief   Implementation of the continuous safety supervision.
 * @copyright MIT License, see LICENSE.
 */

#include "safety_monitor.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

/** @brief Detail value meaning "no extra information". */
#define SAFETY_NO_DETAIL 0u

tp_status_t tp_safety_monitor_init(tp_safety_state_t *st)
{
    if (st == NULL) {
        return TP_ERR_PARAM;
    }
    (void)memset(st, 0, sizeof(*st));
    st->initialised = 1;
    LOG_INFO(TP_MOD_SAFETY,
             "monitor ready: stall %.0f mrad/s for %u ms, overcurrent %.0f mA for %u ms, "
             "comm timeout %u ms",
             (double)TP_STALL_SPEED_THRESHOLD_MRAD_S, (unsigned)TP_STALL_TRIP_MS,
             (double)TP_MAX_CURRENT_MA, (unsigned)TP_OVERCURRENT_TRIP_MS,
             (unsigned)TP_COMM_TIMEOUT_MS);
    return TP_OK;
}

void tp_safety_monitor_reset_stats(tp_safety_state_t *st)
{
    uint32_t checks;

    if (st == NULL) {
        return;
    }
    checks = st->checks_run;
    st->stall_ms = 0u;
    st->overcurrent_ms = 0u;
    st->sensor_bad_count = 0u;
    st->encoder_bad_count = 0u;
    st->last_speed_mrad_s = 0.0f;
    st->max_speed_seen = 0.0f;
    st->max_current_seen = 0.0f;
    st->max_force_seen = 0.0f;
    st->checks_run = checks;
    st->fault_raises = 0u;
    st->estop_events = 0u;
    st->limit_events = 0u;
}

int tp_safety_check_estop(tp_safety_state_t *st, const tp_safety_inputs_t *in,
                          tp_fault_manager_t *fm, uint32_t now_ms)
{
    if ((st == NULL) || (in == NULL) || (fm == NULL) || (in->estop == NULL)) {
        return 0;
    }
    if (tp_limit_switch_is_active(in->estop) == 0) {
        return 0;
    }

    /* The E-stop input is a normally-closed loop: a *broken wire* also reads as
     * asserted.  That is the fail-safe polarity and is exactly why no
     * debouncing is applied here - a 3-sample filter would add 30 ms to a
     * signal whose entire purpose is to be immediate. */
    st->estop_events++;
    if (tp_fault_is_latched(fm, TP_FAULT_ESTOP) == 0) {
        (void)tp_fault_raise(fm, TP_FAULT_ESTOP, SAFETY_NO_DETAIL, now_ms);
        st->fault_raises++;
    }
    /* Kill in hardware, not in software: a software stop still depends on the
     * control task running, and the whole point of an E-stop is to work when
     * nothing else does. */
    tp_platform_emergency_kill();
    return 1;
}

uint32_t tp_safety_monitor_update(tp_safety_state_t *st,
                                  const tp_safety_inputs_t *in,
                                  tp_fault_manager_t *fm, uint32_t now_ms)
{
    uint32_t raised = 0u;
    uint32_t elapsed_ms;
    float force;
    float speed;
    float current;
    float duty;
    uint8_t detail;

    if ((st == NULL) || (in == NULL) || (fm == NULL) || (st->initialised == 0)) {
        return 0u;
    }

    elapsed_ms = (st->last_check_ms == 0u) ? 0u : (now_ms - st->last_check_ms);
    st->last_check_ms = now_ms;
    st->checks_run++;

    /* =====================================================================
     * 0. Emergency stop.  Checked first, and it short-circuits everything:
     *    there is no point evaluating a force limit on a machine whose drives
     *    have already been killed.
     * ===================================================================== */
    if (tp_safety_check_estop(st, in, fm, now_ms) != 0) {
        return 1u;
    }

    /* =====================================================================
     * 1. Over-force.  No debounce: see the header's rationale.
     * ===================================================================== */
    if (in->force_sensor != NULL) {
        force = tp_force_sensor_get_force_n(in->force_sensor);
        if (force > st->max_force_seen) {
            st->max_force_seen = force;
        }

        if (force > TP_FORCE_MAX_N) {
            /* Detail carries the force in centinewtons so the host can display
             * the magnitude of the violation, not just that it happened. */
            detail = (uint8_t)((force * 100.0f > 255.0f) ? 255.0f : force * 100.0f);
            (void)tp_fault_raise(fm, TP_FAULT_FORCE_OVERLOAD, detail, now_ms);
            raised++;
            st->fault_raises++;
        } else if (tp_fault_is_active(fm, TP_FAULT_FORCE_OVERLOAD) != 0) {
            (void)tp_fault_clear_condition(fm, TP_FAULT_FORCE_OVERLOAD, now_ms);
        }

        if (tp_force_sensor_is_faulty(in->force_sensor) != 0) {
            (void)tp_fault_raise(fm, TP_FAULT_SENSOR,
                                 (uint8_t)tp_force_sensor_spike_count(in->force_sensor),
                                 now_ms);
            raised++;
            st->fault_raises++;
        } else if (tp_fault_is_active(fm, TP_FAULT_SENSOR) != 0) {
            (void)tp_fault_clear_condition(fm, TP_FAULT_SENSOR, now_ms);
        }
    }

    /* =====================================================================
     * 2. Arm joint: speed, current, stall, encoder plausibility.
     * ===================================================================== */
    if ((in->joint_encoder != NULL) && (in->joint_motor != NULL)) {
        speed = tp_encoder_get_speed_mrad_s(in->joint_encoder);
        current = tp_motor_get_current_ma(in->joint_motor);
        duty = tp_motor_get_duty(in->joint_motor);

        if (TP_FABS(speed) > st->max_speed_seen) {
            st->max_speed_seen = TP_FABS(speed);
        }
        if (TP_FABS(current) > st->max_current_seen) {
            st->max_current_seen = TP_FABS(current);
        }

        /* --- encoder fault: physically impossible acceleration ---------- */
        if (elapsed_ms > 0u) {
            float accel = TP_FABS(speed - st->last_speed_mrad_s)
                        / ((float)elapsed_ms / 1000.0f);
            if (accel > TP_ENCODER_MAX_ACCEL_MRAD_S2) {
                st->encoder_bad_count++;
                if (st->encoder_bad_count >= 2u) {
                    (void)tp_fault_raise(fm, TP_FAULT_ENCODER,
                                         (uint8_t)(st->encoder_bad_count > 255u
                                                   ? 255u : st->encoder_bad_count),
                                         now_ms);
                    raised++;
                    st->fault_raises++;
                    /* Instant kill: the feedback path is untrustworthy and
                     * continuing to drive closed-loop is indefensible. */
                    tp_platform_emergency_kill();
                }
            } else {
                st->encoder_bad_count = 0u;
            }
        }
        st->last_speed_mrad_s = speed;

        if (tp_encoder_is_faulty(in->joint_encoder) != 0) {
            (void)tp_fault_raise(fm, TP_FAULT_ENCODER,
                                 (uint8_t)tp_encoder_consecutive_errors(in->joint_encoder),
                                 now_ms);
            raised++;
            st->fault_raises++;
        }

        /* --- over-current, debounced ------------------------------------ */
        if (TP_FABS(current) > TP_MAX_CURRENT_MA) {
            st->overcurrent_ms += elapsed_ms;
            if (st->overcurrent_ms >= TP_OVERCURRENT_TRIP_MS) {
                (void)tp_fault_raise(fm, TP_FAULT_OVERCURRENT,
                                     (uint8_t)(TP_FABS(current) / 100.0f > 255.0f
                                               ? 255u
                                               : (uint8_t)(TP_FABS(current) / 100.0f)),
                                     now_ms);
                raised++;
                st->fault_raises++;
                (void)tp_motor_disable(in->joint_motor);
            }
        } else {
            st->overcurrent_ms = 0u;
            if (tp_fault_is_active(fm, TP_FAULT_OVERCURRENT) != 0) {
                (void)tp_fault_clear_condition(fm, TP_FAULT_OVERCURRENT, now_ms);
            }
        }

        /* --- stall, debounced, and only while torque is actually demanded - */
        if ((TP_FABS(duty) > TP_STALL_PWM_THRESHOLD) &&
            (TP_FABS(speed) < TP_STALL_SPEED_THRESHOLD_MRAD_S)) {
            st->stall_ms += elapsed_ms;
            if (st->stall_ms >= TP_STALL_TRIP_MS) {
                (void)tp_fault_raise(fm, TP_FAULT_MOTOR_STALL,
                                     (uint8_t)(TP_FABS(duty) / 10.0f > 255.0f
                                               ? 255u
                                               : (uint8_t)(TP_FABS(duty) / 10.0f)),
                                     now_ms);
                raised++;
                st->fault_raises++;
                (void)tp_motor_disable(in->joint_motor);
            }
        } else {
            st->stall_ms = 0u;
            if (tp_fault_is_active(fm, TP_FAULT_MOTOR_STALL) != 0) {
                (void)tp_fault_clear_condition(fm, TP_FAULT_MOTOR_STALL, now_ms);
            }
        }
    }

    /* =====================================================================
     * 3. Gripper drive: over-current only.  A jaw stall is *normal* at the
     *    mechanical stop, so stall detection is deliberately not applied here;
     *    treating it as a fault would trip on every full close.
     * ===================================================================== */
    if (in->gripper != NULL) {
        tp_motor_t *gm = tp_gripper_motor(in->gripper);
        if (gm != NULL) {
            if (tp_motor_is_faulted(gm) != 0) {
                (void)tp_fault_raise(fm, TP_FAULT_OVERCURRENT,
                                     (uint8_t)(TP_FABS(tp_motor_get_current_ma(gm))
                                               / 100.0f),
                                     now_ms);
                raised++;
                st->fault_raises++;
            }
        }
    }

    /* =====================================================================
     * 4. Limit switches.  A limit is only a fault if the joint is being
     *    commanded *further into it*; merely resting on a limit after homing is
     *    the normal state and must not raise anything.
     * ===================================================================== */
    if (in->limit_min != NULL) {
        if (tp_limit_switch_is_active(in->limit_min) != 0) {
            if ((in->joint_motor != NULL) &&
                (tp_motor_get_duty(in->joint_motor) < -TP_STALL_PWM_THRESHOLD)) {
                (void)tp_fault_raise(fm, TP_FAULT_LIMIT, 0u, now_ms);
                raised++;
                st->fault_raises++;
                st->limit_events++;
            }
        }
    }
    if (in->limit_max != NULL) {
        if (tp_limit_switch_is_active(in->limit_max) != 0) {
            if ((in->joint_motor != NULL) &&
                (tp_motor_get_duty(in->joint_motor) > TP_STALL_PWM_THRESHOLD)) {
                (void)tp_fault_raise(fm, TP_FAULT_LIMIT, 1u, now_ms);
                raised++;
                st->fault_raises++;
                st->limit_events++;
            }
        }
    }

    /* =====================================================================
     * 5. Communication timeout.  A WARNING, not an ERROR: losing the host must
     *    stop the cycle but must not require a power cycle to recover.
     * ===================================================================== */
    if (in->comm_seen != 0) {
        uint32_t silent = now_ms - in->last_comm_ms;
        if (silent > TP_COMM_TIMEOUT_MS) {
            if (tp_fault_is_latched(fm, TP_FAULT_COMM_TIMEOUT) == 0) {
                (void)tp_fault_raise(fm, TP_FAULT_COMM_TIMEOUT,
                                     (uint8_t)((silent / 100u > 255u)
                                               ? 255u : (silent / 100u)),
                                     now_ms);
                raised++;
                st->fault_raises++;
            }
        } else if (tp_fault_is_active(fm, TP_FAULT_COMM_TIMEOUT) != 0) {
            (void)tp_fault_clear_condition(fm, TP_FAULT_COMM_TIMEOUT, now_ms);
            (void)tp_fault_clear(fm, TP_FAULT_COMM_TIMEOUT, now_ms);
        }
    }

    return raised;
}

int tp_safety_is_stalled(const tp_safety_state_t *st)
{
    if (st == NULL) {
        return 0;
    }
    return (st->stall_ms > 0u) ? 1 : 0;
}
