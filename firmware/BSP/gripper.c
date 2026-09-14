/**
 * @file    gripper.c
 * @brief   Implementation of the gripper jaw driver.
 * @copyright MIT License, see LICENSE.
 */

#include "gripper.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

/** @brief Homing timeout, milliseconds. */
#define GRIPPER_HOME_TIMEOUT_MS 3000u

/** @brief Homing speed, mm/s. Slow enough not to shock the mechanism. */
#define GRIPPER_HOME_SPEED_MM_S 15.0f

/**
 * @brief Clamp a float into a range.
 *
 * @param[in] v   Value.
 * @param[in] lo  Lower bound.
 * @param[in] hi  Upper bound.
 * @return Clamped value.
 */
static float gripper_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/**
 * @brief Map a jaw velocity command onto a motor duty.
 *
 * The jaw is a permanent-magnet DC motor with a lead screw, so speed is very
 * nearly proportional to applied voltage over the useful range; the mapping is
 * therefore a pure scale plus a sign inversion.  A production version would add
 * a dead-band compensation term and a feed-forward from the measured force
 * (because the load changes the constant of proportionality), and both are
 * listed in `docs/13_future_work.md`.
 *
 * This is the inverse of the convention modelled in `plant_model.c`; a unit test
 * asserts that the two agree, so the two definitions cannot drift apart.
 *
 * @param[in] mm_s  Jaw velocity in mm/s, negative closes.
 * @return Duty in per-mille, saturating at the bus limit.
 */
static float gripper_velocity_to_duty(float mm_s)
{
    float duty;

    if (!TP_ISFINITE(mm_s)) {
        return 0.0f;
    }
    /* Positive duty closes (see motor.h's sign convention), and positive mm_s
     * opens, hence the negation. */
    duty = -(mm_s / TP_GRIPPER_MAX_SPEED_MM_S) * TP_MAX_PWM_PERMILLE;
    return gripper_clampf(duty, -TP_MAX_PWM_PERMILLE, TP_MAX_PWM_PERMILLE);
}

/**
 * @brief Refresh the limit-switch state from the platform.
 *
 * @param[in,out] g  Instance.
 */
static void gripper_update_limits(tp_gripper_t *g)
{
    uint8_t mask = tp_platform_io_read_all();

    g->limit_open = ((mask >> (int)TP_IO_LIMIT_GRIPPER_OPEN) & 0x1u) ? 1 : 0;
    g->limit_closed = ((mask >> (int)TP_IO_LIMIT_GRIPPER_CLOSE) & 0x1u) ? 1 : 0;
}

tp_status_t tp_gripper_init(tp_gripper_t *g, float dt_s)
{
    tp_status_t st;
    uint32_t start_ms;
    int homed = 0;

    if (g == NULL) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }

    (void)memset(g, 0, sizeof(*g));
    st = tp_motor_init(&g->motor, TP_MOTOR_GRIPPER, dt_s);
    if (st != TP_OK) {
        return st;
    }
    st = tp_encoder_init(&g->encoder, TP_ENCODER_GRIPPER);
    if (st != TP_OK) {
        return st;
    }
    if (tp_encoder_set_zero(&g->encoder) != TP_OK) {
        return TP_ERR_IO;
    }
    g->initialised = 1;
    g->gap_mm = TP_GRIPPER_MAX_MM;

    /* --- homing -------------------------------------------------------
     * A linear jaw has no absolute reference, so drive slowly towards the open
     * stop and declare that position to be "fully open".  Without this the gap
     * reading after a power cycle would be relative to an arbitrary origin and
     * every force threshold would be off by a constant. */
    start_ms = tp_platform_time_ms();
    while ((tp_platform_time_ms() - start_ms) < GRIPPER_HOME_TIMEOUT_MS) {
        gripper_update_limits(g);
        if (g->limit_open != 0) {
            homed = 1;
            break;
        }
        (void)tp_motor_set_duty(&g->motor, gripper_velocity_to_duty(GRIPPER_HOME_SPEED_MM_S));
        (void)tp_motor_update(&g->motor, dt_s);
        (void)tp_encoder_update(&g->encoder, tp_platform_time_ms(),
                                (uint32_t)(dt_s * 1000000.0f));
        g->gap_mm = TP_GRIPPER_MAX_MM
                  - ((float)g->encoder.count / TP_GRIPPER_COUNTS_PER_MM);
        g->gap_mm = gripper_clampf(g->gap_mm, TP_GRIPPER_MIN_MM, TP_GRIPPER_MAX_MM);
    }
    (void)tp_motor_set_duty(&g->motor, 0.0f);
    (void)tp_motor_update(&g->motor, dt_s);

    if (homed == 0) {
        LOG_ERROR(TP_MOD_GRIPPER,
                  "homing failed: open switch never asserted within %lu ms",
                  (unsigned long)GRIPPER_HOME_TIMEOUT_MS);
        return TP_ERR_IO;
    }

    if (tp_encoder_set_zero(&g->encoder) != TP_OK) {
        return TP_ERR_IO;
    }
    g->gap_mm = TP_GRIPPER_MAX_MM;
    LOG_INFO(TP_MOD_GRIPPER, "homed: fully open at %.2f mm", (double)g->gap_mm);
    return TP_OK;
}

tp_status_t tp_gripper_set_velocity_mm_s(tp_gripper_t *g, float mm_s)
{
    if ((g == NULL) || (g->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(mm_s)) {
        return TP_ERR_RANGE;
    }
    if (TP_FABS(mm_s) > (TP_GRIPPER_MAX_SPEED_MM_S + 1.0e-3f)) {
        return TP_ERR_RANGE;
    }
    g->cmd_velocity_mm_s = mm_s;
    g->position_mode = 0;
    return TP_OK;
}

tp_status_t tp_gripper_move_to_mm(tp_gripper_t *g, float target_mm,
                                  float speed_mm_s)
{
    if ((g == NULL) || (g->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(target_mm) || !TP_ISFINITE(speed_mm_s) ||
        (speed_mm_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }
    g->target_mm = gripper_clampf(target_mm, TP_GRIPPER_MIN_MM, TP_GRIPPER_MAX_MM);
    g->cmd_velocity_mm_s = speed_mm_s;
    g->position_mode = 1;
    g->move_count++;
    return TP_OK;
}

tp_status_t tp_gripper_open(tp_gripper_t *g)
{
    return tp_gripper_move_to_mm(g, TP_GRIPPER_MAX_MM, TP_APPROACH_FAST_SPEED_MM_S);
}

tp_status_t tp_gripper_close(tp_gripper_t *g)
{
    /* Position-only baseline: close to the nominal stem position with a fixed
     * gap and no force feedback.  Deliberately kept so Experiment 03 can
     * compare it with the force-controlled path on identical dynamics. */
    return tp_gripper_move_to_mm(g, TP_APPROACH_SLOW_AT_MM,
                                 TP_APPROACH_SLOW_SPEED_MM_S);
}

tp_status_t tp_gripper_stop(tp_gripper_t *g)
{
    if ((g == NULL) || (g->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    g->cmd_velocity_mm_s = 0.0f;
    g->position_mode = 0;
    return TP_OK;
}

tp_status_t tp_gripper_update(tp_gripper_t *g, float dt_s)
{
    float commanded;
    float duty;
    tp_status_t st;
    tp_status_t ret = TP_OK;

    if ((g == NULL) || (g->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }

    gripper_update_limits(g);

    /* --- position mode: a P loop producing a velocity reference ---------- */
    commanded = g->cmd_velocity_mm_s;
    if (g->position_mode != 0) {
        float error_mm = g->target_mm - g->gap_mm;
        commanded = TP_GRIPPER_POS_KP * error_mm;
        /* Cap the P output at the caller's requested speed.  Without the cap a
         * large step (open at 40 mm out) would command a speed the mechanism
         * cannot take and the gain would be meaningless. */
        commanded = gripper_clampf(commanded, -TP_FABS(g->cmd_velocity_mm_s),
                                   TP_FABS(g->cmd_velocity_mm_s));
        if (TP_FABS(error_mm) <= TP_GRIPPER_POS_TOLERANCE_MM) {
            commanded = 0.0f;
            g->position_mode = 0;
        }
    }

    /* --- limit veto ------------------------------------------------------
     * The veto is applied to the *velocity*, not to the duty, so the sign
     * convention is handled once and the caller cannot bypass it. */
    g->veto_active = 0;
    if ((g->limit_open != 0) && (commanded > 0.0f)) {
        commanded = 0.0f;
        g->veto_active = 1;
    }
    if ((g->limit_closed != 0) && (commanded < 0.0f)) {
        commanded = 0.0f;
        g->veto_active = 1;
    }
    if (g->veto_active != 0) {
        g->limit_events++;
    }
    /* The vetoed value is stored separately.  Writing it back into
     * `cmd_velocity_mm_s` would latch the veto into the caller's setpoint and
     * the axis would never move again once it had touched a limit — see the
     * note in gripper.h, which documents the concrete failure this caused. */
    g->applied_velocity_mm_s = commanded;

    /* --- write through the motor abstraction ---------------------------- */
    duty = gripper_velocity_to_duty(commanded);
    g->cmd_duty = duty;
    st = tp_motor_set_duty(&g->motor, duty);
    if (st == TP_ERR_FAULT_ACTIVE) {
        ret = TP_ERR_FAULT_ACTIVE;
    }
    st = tp_motor_update(&g->motor, dt_s);
    if (st == TP_ERR_FAULT_ACTIVE) {
        ret = TP_ERR_FAULT_ACTIVE;
    }

    /* --- feedback ------------------------------------------------------- */
    (void)tp_encoder_update(&g->encoder, tp_platform_time_ms(),
                            (uint32_t)(dt_s * 1000000.0f));
    g->gap_mm = TP_GRIPPER_MAX_MM
              - ((float)g->encoder.count / TP_GRIPPER_COUNTS_PER_MM);
    g->gap_mm = gripper_clampf(g->gap_mm, TP_GRIPPER_MIN_MM, TP_GRIPPER_MAX_MM);
    g->velocity_mm_s = -tp_encoder_get_speed_mrad_s(&g->encoder)
                     / (TP_MRAD_PER_COUNT * TP_GRIPPER_COUNTS_PER_MM);
    if (!TP_ISFINITE(g->velocity_mm_s)) {
        g->velocity_mm_s = 0.0f;
    }

    /* --- stall detection for a position move ----------------------------- */
    /* A commanded move that produces no motion for a while means the jaw met
     * something it cannot push (a branch, or a jammed lead screw).  The driver
     * reports it; the safety layer decides what it means. */
    if ((g->position_mode != 0) &&
        (TP_FABS(g->cmd_velocity_mm_s) > 0.5f) &&
        (TP_FABS(g->velocity_mm_s) < 0.2f) &&
        (TP_FABS(g->motor.current_ma) > (0.4f * TP_MAX_CURRENT_MA))) {
        g->stall_events++;
    }
    return ret;
}

float tp_gripper_get_gap_mm(const tp_gripper_t *g)
{
    return (g != NULL) ? g->gap_mm : 0.0f;
}

float tp_gripper_get_velocity_mm_s(const tp_gripper_t *g)
{
    return (g != NULL) ? g->velocity_mm_s : 0.0f;
}

int tp_gripper_is_at_target(const tp_gripper_t *g)
{
    if (g == NULL) {
        return 0;
    }
    if (g->position_mode == 0) {
        return 1;
    }
    return (TP_FABS(g->target_mm - g->gap_mm) <= TP_GRIPPER_POS_TOLERANCE_MM)
         ? 1 : 0;
}

int tp_gripper_is_open(const tp_gripper_t *g)
{
    if (g == NULL) {
        return 0;
    }
    /* "Open" means the jaw is at or on the open stop.  Testing the limit switch
     * as well as the measured gap matters: the switch trips a fraction of a
     * millimetre *before* the mechanical end of travel, so the gap never quite
     * reaches TP_GRIPPER_MAX_MM and a gap-only test would report "not open" for
     * a jaw that is physically as far open as it can go.  The tolerance is
     * deliberately wider than the switch's own trigger band. */
    if (g->limit_open != 0) {
        return 1;
    }
    return (g->gap_mm >= (TP_GRIPPER_MAX_MM
                          - (2.0f * TP_GRIPPER_POS_TOLERANCE_MM))) ? 1 : 0;
}

int tp_gripper_at_limit(const tp_gripper_t *g)
{
    if (g == NULL) {
        return 0;
    }
    return ((g->limit_open != 0) || (g->limit_closed != 0)) ? 1 : 0;
}

tp_motor_t *tp_gripper_motor(tp_gripper_t *g)
{
    return (g != NULL) ? &g->motor : NULL;
}

tp_encoder_t *tp_gripper_encoder(tp_gripper_t *g)
{
    return (g != NULL) ? &g->encoder : NULL;
}
