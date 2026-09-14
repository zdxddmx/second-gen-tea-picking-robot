/**
 * @file    app_tasks.c
 * @brief   Wiring, task bodies and command handlers.
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
 * Protocol command handlers
 * ========================================================================= */

/** @brief `HEARTBEAT` handler. */
static tp_status_t on_heartbeat(void *user, uint32_t seq)
{
    tp_app_context_t *app = (tp_app_context_t *)user;

    (void)tp_protocol_send_heartbeat_ack(&app->proto, seq);
    return TP_OK;
}

/** @brief `SET_TARGET` handler. */
static tp_status_t on_set_target(void *user, const tp_payload_set_target_t *t)
{
    tp_app_context_t *app = (tp_app_context_t *)user;

    if (t == NULL) {
        return TP_ERR_PARAM;
    }
    return tp_app_inject_target(app, t->x_mm, t->y_mm, t->z_mm, t->confidence,
                                t->target_id);
}

/** @brief `START_PICK` handler. */
static tp_status_t on_start_pick(void *user, uint16_t target_id)
{
    tp_app_context_t *app = (tp_app_context_t *)user;

    if ((target_id != 0u) && (app->target.id != target_id)) {
        /* The host asked for a specific bud that is not the latched one.  That
         * is a real race (the vision system may have moved on), so it is
         * reported rather than silently substituted. */
        LOG_WARN(TP_MOD_ROBOT, "START_PICK for target %u but %u is latched",
                 (unsigned)target_id, (unsigned)app->target.id);
        return TP_ERR_STATE;
    }
    return tp_app_start_pick(app);
}

/** @brief `STOP` handler. */
static tp_status_t on_stop(void *user)
{
    return tp_app_stop((tp_app_context_t *)user);
}

/** @brief `RESET` handler. */
static tp_status_t on_reset(void *user)
{
    return tp_app_reset((tp_app_context_t *)user);
}

/** @brief `GET_STATUS` handler. */
static tp_status_t on_get_status(void *user, tp_payload_status_t *out)
{
    tp_app_context_t *app = (tp_app_context_t *)user;

    if (out == NULL) {
        return TP_ERR_PARAM;
    }
    out->state = (uint8_t)tp_fsm_state(&app->fsm);
    out->fault_code = (uint8_t)tp_fault_highest(&app->faults);
    out->pos_mrad = (int32_t)app->meas_position_mrad;
    out->vel_mrad_s = (int16_t)tp_fixed_from_float(app->meas_velocity_mrad_s,
                                                   TP_WIRE_MRAD_S, -32768, 32767);
    out->force_mn = (int16_t)tp_fixed_from_float(app->meas_force_n,
                                                 TP_WIRE_N_TO_MN, -32768, 32767);
    out->pwm_permille = (int16_t)tp_fixed_from_float(
        tp_motor_get_duty(&app->joint_motor), TP_WIRE_PERMILLE, -32768, 32767);
    out->current_ma = (int16_t)tp_fixed_from_float(app->meas_current_ma,
                                                   TP_WIRE_MA, -32768, 32767);
    out->cycle_count = (uint16_t)((app->cycle_count > 65535u) ? 65535u
                                                              : app->cycle_count);
    return TP_OK;
}

/** @brief `GET_SENSOR` handler. */
static tp_status_t on_get_sensor(void *user, tp_payload_sensor_t *out)
{
    tp_app_context_t *app = (tp_app_context_t *)user;

    if (out == NULL) {
        return TP_ERR_PARAM;
    }
    out->pos_mrad = (int32_t)app->meas_position_mrad;
    out->vel_mrad_s = (int16_t)tp_fixed_from_float(app->meas_velocity_mrad_s,
                                                   TP_WIRE_MRAD_S, -32768, 32767);
    out->force_mn = (int16_t)tp_fixed_from_float(
        tp_force_sensor_get_force_n(&app->force), TP_WIRE_N_TO_MN, -32768, 32767);
    out->adc_raw = tp_force_sensor_get_counts(&app->force);
    out->current_ma = (int16_t)tp_fixed_from_float(app->meas_current_ma,
                                                   TP_WIRE_MA, -32768, 32767);
    /* No temperature sensor is fitted on the reference design; reporting a
     * fabricated value would be worse than reporting the sentinel the host
     * already understands as "not measured". */
    out->temp_c_x10 = (int16_t)TP_SENSOR_TEMP_UNAVAILABLE;
    return TP_OK;
}

/** @brief `GET_FAULT` handler. */
static tp_status_t on_get_fault(void *user, tp_payload_fault_t *out)
{
    tp_app_context_t *app = (tp_app_context_t *)user;
    tp_fault_record_t rec;

    if (out == NULL) {
        return TP_ERR_PARAM;
    }
    out->fault_code = (uint8_t)tp_fault_highest(&app->faults);
    out->detail = 0u;
    out->timestamp_ms = tp_platform_time_ms();
    if (tp_fault_get_record(&app->faults, tp_fault_highest(&app->faults),
                            &rec) == TP_OK) {
        out->timestamp_ms = rec.last_ms;
        out->detail = rec.detail;
    }
    return TP_OK;
}

/** @brief `SET_PID` handler. */
static tp_status_t on_set_pid(void *user, const tp_payload_pid_t *p)
{
    tp_app_context_t *app = (tp_app_context_t *)user;
    tp_status_t st;

    if (p == NULL) {
        return TP_ERR_PARAM;
    }
    /* The wire loop identifiers and the cascade's own selectors are declared to
     * match numerically, so the mapping is the identity.  It is still written
     * explicitly, and the default case still returns an error, so that adding a
     * fourth loop on one side and not the other is a compile-visible decision
     * rather than a silent misroute. */
    switch ((tp_pid_loop_t)p->loop_id) {
    case TP_PID_LOOP_POSITION:
        st = tp_cascade_set_gains(&app->cascade, TP_CASCADE_LOOP_POSITION,
                                 p->kp, p->ki, p->kd);
        break;
    case TP_PID_LOOP_VELOCITY:
        st = tp_cascade_set_gains(&app->cascade, TP_CASCADE_LOOP_VELOCITY,
                                 p->kp, p->ki, p->kd);
        break;
    case TP_PID_LOOP_FORCE:
        st = tp_force_control_set_gains(&app->grip_ctl, p->kp, p->ki, p->kd);
        break;
    default:
        return TP_ERR_RANGE;
    }
    if (st == TP_OK) {
        LOG_INFO(TP_MOD_CONTROL, "loop %u gains set to Kp=%.4f Ki=%.4f Kd=%.4f",
                 (unsigned)p->loop_id, (double)p->kp, (double)p->ki,
                 (double)p->kd);
    }
    return st;
}

/** @brief `GET_PID` handler. */
static tp_status_t on_get_pid(void *user, uint8_t loop_id, tp_payload_pid_t *out)
{
    tp_app_context_t *app = (tp_app_context_t *)user;
    tp_status_t st = TP_ERR_RANGE;

    if (out == NULL) {
        return TP_ERR_PARAM;
    }
    out->loop_id = loop_id;
    switch ((tp_pid_loop_t)loop_id) {
    case TP_PID_LOOP_POSITION:
        st = tp_cascade_get_gains(&app->cascade, TP_CASCADE_LOOP_POSITION,
                                 &out->kp, &out->ki, &out->kd);
        break;
    case TP_PID_LOOP_VELOCITY:
        st = tp_cascade_get_gains(&app->cascade, TP_CASCADE_LOOP_VELOCITY,
                                 &out->kp, &out->ki, &out->kd);
        break;
    case TP_PID_LOOP_FORCE:
        st = tp_force_control_get_gains(&app->grip_ctl, &out->kp, &out->ki,
                                       &out->kd);
        break;
    default:
        break;
    }
    return st;
}

/** @brief `SET_FORCE_TARGET` handler. */
static tp_status_t on_set_force_target(void *user, uint16_t force_mn)
{
    tp_app_context_t *app = (tp_app_context_t *)user;
    float n = tp_fixed_to_float((int32_t)force_mn, TP_WIRE_N_TO_MN);

    return tp_force_control_set_force(&app->grip_ctl, n);
}

/** @brief `SET_MODE` handler. */
static tp_status_t on_set_mode(void *user, uint8_t mode)
{
    tp_app_context_t *app = (tp_app_context_t *)user;

    if (mode > (uint8_t)TP_MODE_CALIBRATE) {
        return TP_ERR_RANGE;
    }
    app->mode = (tp_robot_mode_t)mode;
    LOG_INFO(TP_MOD_ROBOT, "mode set to %u", (unsigned)mode);
    return TP_OK;
}

/** @brief `EMERGENCY_STOP` handler. */
static tp_status_t on_emergency_stop(void *user)
{
    return tp_app_emergency_stop((tp_app_context_t *)user);
}

/** @brief `CLEAR_FAULT` handler. */
static tp_status_t on_clear_fault(void *user)
{
    tp_app_context_t *app = (tp_app_context_t *)user;
    uint32_t n = tp_fault_clear_all(&app->faults, tp_platform_time_ms());

    (void)tp_force_control_clear_fault(&app->grip_ctl);
    if (n > 0u) {
        LOG_INFO(TP_MOD_SAFETY, "%lu fault(s) cleared by host command",
                 (unsigned long)n);
    }
    /* Only release the hardware kill once nothing critical is latched; doing it
     * unconditionally would let a CLEAR_FAULT command re-energise a drive whose
     * fault is still physically present. */
    if (tp_fault_requires_kill(&app->faults) == 0) {
        tp_platform_emergency_release();
    }
    tp_fsm_set_fault_state(&app->fsm, tp_fault_any_latched(&app->faults));
    if (tp_fault_any_latched(&app->faults) == 0) {
        (void)tp_fsm_handle_event(&app->fsm, TP_EV_RESET,
                                  tp_platform_time_ms());
    }
    return (tp_fault_any_latched(&app->faults) == 0) ? TP_OK
                                                     : TP_ERR_FAULT_ACTIVE;
}

/** @brief `CALIBRATE_FORCE` handler. */
static tp_status_t on_calibrate_force(void *user, uint8_t kind)
{
    tp_app_context_t *app = (tp_app_context_t *)user;

    if (kind == 0u) {
        /* Zero calibration requires the jaws to be open and unloaded; refusing
         * if a cycle is in progress prevents the operator from baking a grip
         * force into the offset. */
        if (tp_fsm_state(&app->fsm) != TP_STATE_IDLE) {
            LOG_WARN(TP_MOD_FORCE, "zero calibration refused: robot is in %s",
                     tp_state_name(tp_fsm_state(&app->fsm)));
            return TP_ERR_STATE;
        }
        (void)tp_gripper_open(&app->gripper);
        return tp_force_sensor_zero_calibrate(&app->force);
    }
    /* kind 1 is a scale calibration, which needs a known load applied by hand;
     * it is driven from the monitor tool with the mass already hanging. */
    return TP_ERR_UNSUPPORTED;
}

/** @brief Link-loss handler. */
static void on_link_lost(void *user, uint32_t silent_ms)
{
    tp_app_context_t *app = (tp_app_context_t *)user;

    (void)silent_ms;
    /* A lost host must stop the cycle, not merely be noted: continuing to pick
     * on a target that the vision system has since re-evaluated is how a robot
     * picks the wrong bud.  The fault itself is raised by the safety monitor so
     * that there is exactly one place where link loss becomes a fault. */
    if (tp_fsm_state(&app->fsm) != TP_STATE_IDLE) {
        LOG_WARN(TP_MOD_ROBOT, "host lost mid-cycle: stopping and opening jaws");
        (void)tp_app_stop(app);
    }
}

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

tp_status_t tp_app_init(tp_app_context_t *app)
{
    tp_protocol_ops_t ops;
    tp_status_t st;
    tp_status_t first_error = TP_OK;
    uint32_t now;

    if (app == NULL) {
        return TP_ERR_PARAM;
    }
    (void)memset(app, 0, sizeof(*app));
    app->mode = TP_MODE_IDLE;

    /* --- 1. platform and logger --------------------------------------- */
    st = tp_platform_init();
    if (st != TP_OK) {
        LOG_ERROR(TP_MOD_MAIN, "platform init failed: %s", tp_status_str(st));
        return st;
    }

    /* --- 2. safety inputs, checked before anything can be energised ----- */
    if (tp_limit_switch_init(&app->sw_estop, TP_IO_ESTOP, 1) != TP_OK) {
        return TP_ERR_IO; /* E-stop is not optional */
    }
    /* Poll it a few times so the debouncer settles before the self-check reads
     * it; reading a freshly initialised input would report the raw level. */
    {
        int i;
        for (i = 0; i < 5; ++i) {
            (void)tp_limit_switch_update(&app->sw_estop);
        }
    }
    if (tp_limit_switch_is_active(&app->sw_estop) != 0) {
        /* Do not energise anything, do not even finish bringing up the drives:
         * a robot whose E-stop is pressed at boot must stay inert. */
        tp_platform_emergency_kill();
        LOG_ERROR(TP_MOD_MAIN,
                  "E-stop asserted at boot: staying inert until it is released");
        return TP_ERR_FAULT_ACTIVE;
    }
    (void)tp_limit_switch_init(&app->sw_limit_min, TP_IO_LIMIT_JOINT_MIN, 0);
    (void)tp_limit_switch_init(&app->sw_limit_max, TP_IO_LIMIT_JOINT_MAX, 0);

    /* --- 3. sensors and actuators -------------------------------------- */
    st = tp_encoder_init(&app->joint_encoder, TP_ENCODER_JOINT);
    if (st != TP_OK) {
        first_error = st;
    }
    st = tp_motor_init(&app->joint_motor, TP_MOTOR_JOINT, TP_CONTROL_DT_S);
    if ((st != TP_OK) && (first_error == TP_OK)) {
        first_error = st;
    }
    st = tp_force_sensor_init(&app->force, 1.0f / (float)TP_SENSOR_FREQ_HZ);
    if ((st != TP_OK) && (first_error == TP_OK)) {
        first_error = st;
    }
    st = tp_gripper_init(&app->gripper, TP_CONTROL_DT_S);
    if ((st != TP_OK) && (first_error == TP_OK)) {
        first_error = st;
    }
    (void)tp_adc_channel_init(&app->adc_force, TP_ADC_FORCE, TP_SENSOR_DT_S,
                              1.0f, 1000u);
    (void)tp_adc_channel_init(&app->adc_vbus, TP_ADC_VBUS, TP_SENSOR_DT_S,
                              10.0f, 1000u);

    /* --- 4. controllers, supervisors, protocol ------------------------- */
    st = tp_cascade_init(&app->cascade, TP_CONTROL_DT_S);
    if ((st != TP_OK) && (first_error == TP_OK)) {
        first_error = st;
    }
    st = tp_force_control_init(&app->grip_ctl, TP_CONTROL_DT_S);
    if ((st != TP_OK) && (first_error == TP_OK)) {
        first_error = st;
    }
    st = tp_trajectory_init(&app->lift_traj, TP_PROFILE_SCURVE,
                            TP_JOINT_HOME_MRAD, TP_ROBOT_DT_S);
    if ((st != TP_OK) && (first_error == TP_OK)) {
        first_error = st;
    }
    (void)tp_trajectory_set_limits(&app->lift_traj, 20000.0f, 80000.0f, 800000.0f);

    (void)tp_fault_manager_init(&app->faults);
    (void)tp_safety_monitor_init(&app->safety);
    (void)tp_fsm_init(&app->fsm, app);

    app->safety_in.joint_encoder = &app->joint_encoder;
    app->safety_in.joint_motor = &app->joint_motor;
    app->safety_in.gripper = &app->gripper;
    app->safety_in.force_sensor = &app->force;
    app->safety_in.estop = &app->sw_estop;
    app->safety_in.limit_min = &app->sw_limit_min;
    app->safety_in.limit_max = &app->sw_limit_max;
    app->safety_in.comm_seen = 0;

    if (tp_uart_init(&app->link_host, TP_BUS_VISION_UART, TP_UART_VISION_BAUD)
        != TP_OK) {
        LOG_WARN(TP_MOD_COMM, "host link unavailable; running without a host");
    }
    if (tp_uart_init(&app->link_debug, TP_BUS_DEBUG_UART, TP_UART_DEBUG_BAUD)
        != TP_OK) {
        LOG_WARN(TP_MOD_COMM, "debug UART unavailable");
    }
    (void)tp_rs485_init(&app->rs485, TP_BUS_RS485, TP_RS485_BAUD,
                        TP_MODBUS_SLAVE_ADDR);
    (void)tp_can_init(&app->can, TP_CAN_BITRATE_BPS);

    (void)memset(&ops, 0, sizeof(ops));
    ops.user = app;
    ops.on_heartbeat = on_heartbeat;
    ops.on_set_target = on_set_target;
    ops.on_start_pick = on_start_pick;
    ops.on_stop = on_stop;
    ops.on_reset = on_reset;
    ops.on_get_status = on_get_status;
    ops.on_get_sensor = on_get_sensor;
    ops.on_get_fault = on_get_fault;
    ops.on_set_pid = on_set_pid;
    ops.on_get_pid = on_get_pid;
    ops.on_set_force_target = on_set_force_target;
    ops.on_set_mode = on_set_mode;
    ops.on_emergency_stop = on_emergency_stop;
    ops.on_clear_fault = on_clear_fault;
    ops.on_calibrate_force = on_calibrate_force;
    ops.on_link_lost = on_link_lost;
    (void)tp_protocol_init(&app->proto, &app->link_host, &ops);

    now = tp_platform_time_ms();
    LOG_INFO(TP_MOD_MAIN, "application initialised at t=%lu ms (OSAL backend: %s)",
             (unsigned long)now, osal_backend_name());

    /* --- 5. self-check -------------------------------------------------- */
    app->boot_ok = tp_app_self_check(app);
    app->self_check_done = 1;
    (void)tp_fsm_handle_event(&app->fsm,
                              (app->boot_ok != 0) ? TP_EV_SELF_CHECK_PASS
                                                  : TP_EV_SELF_CHECK_FAIL,
                              tp_platform_time_ms());
    if (first_error != TP_OK) {
        LOG_WARN(TP_MOD_MAIN, "initialisation completed with error: %s",
                 tp_status_str(first_error));
    }
    app->initialised = 1;
    return (first_error == TP_OK) ? (app->boot_ok != 0 ? TP_OK
                                                       : TP_ERR_FAULT_ACTIVE)
                                  : first_error;
}

int tp_app_self_check(tp_app_context_t *app)
{
    char faults[128];
    int ok = 1;
    uint32_t now;

    if (app == NULL) {
        return 0;
    }
    now = tp_platform_time_ms();

    /* --- watchdog reset cause ------------------------------------------ */
    if (tp_platform_watchdog_caused_reset() != 0) {
        /* Refuse to auto-resume: a watchdog reset means something stopped
         * feeding it, and starting motion without knowing what is exactly the
         * behaviour the watchdog exists to prevent. */
        (void)tp_fault_raise(&app->faults, TP_FAULT_WATCHDOG, 0u, now);
        LOG_ERROR(TP_MOD_MAIN, "previous reset was caused by the watchdog");
        ok = 0;
    }

    /* --- E-stop polarity ------------------------------------------------ */
    if (tp_limit_switch_is_active(&app->sw_estop) != 0) {
        (void)tp_fault_raise(&app->faults, TP_FAULT_ESTOP, 0u, now);
        ok = 0;
    }

    /* --- limit switches -------------------------------------------------- */
    {
        int i;
        for (i = 0; i < 5; ++i) {
            (void)tp_limit_switch_update(&app->sw_limit_min);
            (void)tp_limit_switch_update(&app->sw_limit_max);
        }
        if ((tp_limit_switch_is_active(&app->sw_limit_min) != 0) &&
            (tp_limit_switch_is_active(&app->sw_limit_max) != 0)) {
            /* Both limits asserted at once is electrically impossible on a
             * working machine, so it means a wiring fault, not a position. */
            (void)tp_fault_raise(&app->faults, TP_FAULT_LIMIT, 2u, now);
            LOG_ERROR(TP_MOD_MAIN, "both arm limits asserted: wiring fault");
            ok = 0;
        }
    }

    /* --- encoder responsiveness ----------------------------------------- */
    {
        float p0 = tp_encoder_get_position_mrad(&app->joint_encoder);
        if (!TP_ISFINITE(p0)) {
            (void)tp_fault_raise(&app->faults, TP_FAULT_ENCODER, 0u, now);
            ok = 0;
        }
        if (tp_encoder_is_faulty(&app->joint_encoder) != 0) {
            (void)tp_fault_raise(&app->faults, TP_FAULT_ENCODER, 1u, now);
            ok = 0;
        }
    }

    /* --- force sensor ---------------------------------------------------- */
    {
        uint16_t counts = 0u;
        if (tp_platform_adc_read(TP_ADC_FORCE, &counts) != TP_OK) {
            (void)tp_fault_raise(&app->faults, TP_FAULT_SENSOR, 0u, now);
            ok = 0;
        } else if ((counts <= (uint16_t)TP_SENSOR_ADC_MIN_COUNTS) ||
                   (counts >= (uint16_t)TP_SENSOR_ADC_MAX_COUNTS)) {
            /* Pinned at a rail means the bridge or the amplifier is not
             * connected; a real cell at rest sits near mid-scale. */
            (void)tp_fault_raise(&app->faults, TP_FAULT_SENSOR, 1u, now);
            LOG_ERROR(TP_MOD_MAIN,
                      "force ADC pinned at %u counts: check the load cell wiring",
                      (unsigned)counts);
            ok = 0;
        }
    }

    /* --- supply rail ------------------------------------------------------ */
    {
        uint16_t vbus = 0u;
        if (tp_platform_adc_read(TP_ADC_VBUS, &vbus) == TP_OK) {
            float volts = tp_adc_channel_counts_to_volts(&app->adc_vbus, vbus);
            LOG_INFO(TP_MOD_MAIN, "supply rail %.1f V", (double)volts);
            if (volts < 18.0f) {
                LOG_WARN(TP_MOD_MAIN,
                         "supply %.1f V is below the 18 V minimum: motion may be "
                         "unreliable", (double)volts);
            }
        }
    }

    /* --- CAN controller --------------------------------------------------- */
    if (tp_can_is_bus_off(&app->can) != 0) {
        LOG_WARN(TP_MOD_MAIN, "CAN controller is in bus-off at boot");
    }

    (void)tp_fault_format_active(&app->faults, faults, sizeof(faults));
    if (ok != 0) {
        LOG_INFO(TP_MOD_MAIN, "self-check PASSED");
    } else {
        LOG_ERROR(TP_MOD_MAIN, "self-check FAILED: %s", faults);
    }
    return ok;
}

tp_status_t tp_app_create_tasks(tp_app_context_t *app)
{
    tp_status_t st;

    if (app == NULL) {
        return TP_ERR_PARAM;
    }

    /* Registration order is irrelevant to the bare-metal scheduler, which sorts
     * by priority, but it is kept in priority order anyway so that the two
     * backends' logs can be diffed. */
    st = osal_task_create(NULL, "control", tp_app_control_step, app,
                          OSAL_PRIO_CRITICAL, TP_STACK_CONTROL,
                          TP_CONTROL_PERIOD_MS);
    if (st != TP_OK) {
        return st;
    }
    st = osal_task_create(NULL, "safety", tp_app_safety_step, app,
                          OSAL_PRIO_REALTIME, TP_STACK_SAFETY, TP_SAFETY_PERIOD_MS);
    if (st != TP_OK) {
        return st;
    }
    st = osal_task_create(NULL, "sensor", tp_app_sensor_step, app,
                          OSAL_PRIO_HIGH, TP_STACK_SENSOR, TP_SENSOR_PERIOD_MS);
    if (st != TP_OK) {
        return st;
    }
    st = osal_task_create(NULL, "robot", tp_app_robot_step, app,
                          OSAL_PRIO_ABOVE_NORMAL, TP_STACK_ROBOT,
                          TP_ROBOT_PERIOD_MS);
    if (st != TP_OK) {
        return st;
    }
    st = osal_task_create(NULL, "comm", tp_app_comm_step, app, OSAL_PRIO_NORMAL,
                          TP_STACK_COMM, TP_COMM_PERIOD_MS);
    if (st != TP_OK) {
        return st;
    }
    st = osal_task_create(NULL, "telemetry", tp_app_telemetry_step, app,
                          OSAL_PRIO_BELOW_NORMAL, TP_STACK_TELEMETRY,
                          TP_TELEMETRY_PERIOD_MS);
    if (st != TP_OK) {
        return st;
    }
    st = osal_task_create(NULL, "debug", tp_app_debug_step, app, OSAL_PRIO_LOW,
                          TP_STACK_DEBUG, TP_DEBUG_PERIOD_MS);
    if (st != TP_OK) {
        return st;
    }
    return TP_OK;
}

/**
 * @brief Initialise the seven periodic timing helpers.
 *
 * Each task owns its own helper so that one task's phase can be perturbed (as a
 * test does) without touching the others, and so that a missed deadline is
 * attributable to a specific task.
 *
 * @param[in,out] app  Context.
 */
static void init_periods(tp_app_context_t *app)
{
    osal_period_init(&app->period_control, TP_CONTROL_PERIOD_MS);
    osal_period_init(&app->period_sensor, TP_SENSOR_PERIOD_MS);
    osal_period_init(&app->period_comm, TP_COMM_PERIOD_MS);
    osal_period_init(&app->period_robot, TP_ROBOT_PERIOD_MS);
    osal_period_init(&app->period_safety, TP_SAFETY_PERIOD_MS);
    osal_period_init(&app->period_telemetry, TP_TELEMETRY_PERIOD_MS);
    osal_period_init(&app->period_debug, TP_DEBUG_PERIOD_MS);
}

tp_status_t tp_app_start_ctx(tp_app_context_t *app)
{
    tp_status_t st;

    if (app == NULL) {
        return TP_ERR_PARAM;
    }
    st = tp_app_init(app);
    if (st != TP_OK) {
        /* Even a failed init goes on to start the scheduler: the fault must be
         * reportable over the link, and the operator needs a way to see and
         * clear it.  The FSM is already in ERROR, so nothing can move. */
        LOG_WARN(TP_MOD_MAIN, "init returned %s; starting anyway to report",
                 tp_status_str(st));
    }
    init_periods(app);
    st = tp_app_create_tasks(app);
    if (st != TP_OK) {
        LOG_ERROR(TP_MOD_MAIN, "task creation failed: %s", tp_status_str(st));
        return st;
    }
    st = tp_platform_watchdog_init(TP_WDG_TIMEOUT_MS);
    if (st != TP_OK) {
        LOG_WARN(TP_MOD_MAIN, "watchdog unavailable (%s)", tp_status_str(st));
    }
    return osal_start();
}

void tp_app_shutdown(tp_app_context_t *app)
{
    if (app == NULL) {
        return;
    }
    /* Shutdown is a deliberate, orderly stop, so the drives are ramped and then
     * disabled rather than emergency-killed.  Calling the emergency path here
     * would put a red ERROR line at the end of every clean run, which trains a
     * reader to ignore the one line that matters in a genuine fault.  The
     * hardware kill is still used when a fault demands it. */
    if (tp_fault_requires_kill(&app->faults) != 0) {
        tp_platform_emergency_kill();
    } else {
        (void)tp_motor_stop(&app->joint_motor, 100u);
        (void)tp_motor_disable(&app->joint_motor);
        (void)tp_motor_disable(tp_gripper_motor(&app->gripper));
        LOG_INFO(TP_MOD_MAIN, "shutdown: drives disabled");
    }
    osal_request_stop();
}

tp_status_t tp_app_set_mode(tp_app_context_t *app, tp_robot_mode_t mode)
{
    if (app == NULL) {
        return TP_ERR_PARAM;
    }
    if (mode > TP_MODE_CALIBRATE) {
        return TP_ERR_RANGE;
    }
    if (app->mode != mode) {
        LOG_INFO(TP_MOD_ROBOT, "mode %u -> %u", (unsigned)app->mode,
                 (unsigned)mode);
        /* Leaving CALIBRATE or MANUAL must not carry a stale integrator into
         * the next automatic cycle. */
        tp_cascade_reset(&app->cascade);
    }
    app->mode = mode;
    return TP_OK;
}

/**
 * @brief The process-wide application context.
 *
 * Static storage rather than a heap allocation for three reasons: the memory
 * map stays analysable (`arm-none-eabi-nm` answers "how much RAM does the
 * application use?" in one line), there is no allocation-failure path at boot,
 * and `.bss` is zero-initialised by the C runtime before `main` runs, so the
 * context cannot contain garbage even if initialisation is interrupted.
 */
static tp_app_context_t s_app;

tp_app_context_t *tp_app_context(void)
{
    return &s_app;
}

tp_status_t tp_app_set_grip_mode(tp_app_context_t *app, int position_only)
{
    if (app == NULL) {
        return TP_ERR_PARAM;
    }
    if (tp_fsm_state(&app->fsm) != TP_STATE_IDLE) {
        LOG_WARN(TP_MOD_ROBOT,
                 "grip mode change refused: robot is in %s",
                 tp_state_name(tp_fsm_state(&app->fsm)));
        return TP_ERR_STATE;
    }
    if (app->grip_position_only != (position_only != 0)) {
        LOG_INFO(TP_MOD_ROBOT, "gripping mode: %s",
                 (position_only != 0)
                     ? "POSITION-ONLY (Experiment 03 baseline, no force feedback)"
                     : "force controlled");
    }
    app->grip_position_only = (position_only != 0) ? 1 : 0;
    return TP_OK;
}

void tp_app_start(void)
{
    (void)tp_app_start_ctx(&s_app);
}
