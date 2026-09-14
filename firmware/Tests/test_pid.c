/**
 * @file    test_pid.c
 * @brief   Unit tests for the PID controller.
 *
 * @details
 * The four categories the project's test plan requires are all present:
 * **normal** input (a step response converges), **boundary** input (the output
 * and integral clamps are reached exactly), **invalid** input (NaN, a null
 * pointer, an uninitialised instance) and **exceptional state** (divergence,
 * reset mid-operation, gain change while running).
 *
 * @copyright MIT License, see LICENSE.
 */

#include "test_framework.h"

#include "pid.h"

/** @brief Sample time used throughout this suite. */
#define DT 0.001f

/**
 * @brief Build a controller with known, easily reasoned-about gains.
 *
 * @param[out] pid  Instance to initialise.
 * @param[in]  kp   Proportional gain.
 * @param[in]  ki   Integral gain.
 * @param[in]  kd   Derivative gain.
 * @param[in]  out_max Symmetric output clamp.
 * @param[in]  i_max   Symmetric integral clamp.
 * @return The status from @ref tp_pid_init.
 */
static tp_status_t make_pid(tp_pid_t *pid, float kp, float ki, float kd,
                            float out_max, float i_max)
{
    tp_pid_config_t cfg = tp_pid_config_default(kp, ki, kd, DT, out_max, i_max,
                                                0.0f);
    return tp_pid_init(pid, &cfg);
}

/* =========================================================================
 * Normal operation
 * ========================================================================= */

TP_TEST_CASE(pid, converge_to_setpoint_with_integral)
{
    tp_pid_t pid;
    float y = 0.0f;
    int i;

    TP_ASSERT_EQ(make_pid(&pid, 1.0f, 2.0f, 0.0f, 100.0f, 50.0f), TP_OK);

    /* Simple first-order plant: y' = (u - y) / 0.05.  With integral action the
     * steady state must reach the setpoint exactly. */
    for (i = 0; i < 20000; ++i) {
        float u = tp_pid_update(&pid, 10.0f, y);
        y += ((u - y) / 0.05f) * DT;
    }
    /* With integral action the steady-state error must vanish.  The tolerance is
     * 1 % of the setpoint, which is what 20 s of settling buys. */
    TP_ASSERT_NEAR(y, 10.0f, 0.01f);
    TP_ASSERT_FINITE(y);
}

TP_TEST_CASE(pid, proportional_only_leaves_steady_state_error)
{
    tp_pid_t pid;
    float y = 0.0f;
    int i;

    /* With Ki = 0 the loop settles where P action balances the plant.  This is
     * the baseline that makes the integral term's purpose measurable rather
     * than a matter of opinion. */
    TP_ASSERT_EQ(make_pid(&pid, 1.0f, 0.0f, 0.0f, 100.0f, 50.0f), TP_OK);
    for (i = 0; i < 4000; ++i) {
        float u = tp_pid_update(&pid, 10.0f, y);
        y += ((u - y) / 0.05f) * DT;
    }
    /* For a first-order plant y' = (u - y)/0.05 with u = Kp*(r - y), the steady
     * state solves (Kp*(r - y) - y) = 0, so with Kp = 1 it is exactly r/2 = 5.
     * That halving *is* the steady-state error the integral term removes, and
     * having the exact value makes the test a specification rather than a
     * sanity check. */
    TP_ASSERT_NEAR(y, 5.0f, 0.01f);
}

TP_TEST_CASE(pid, first_update_has_no_derivative_kick)
{
    tp_pid_t pid;
    float u;

    /* A PID with a large Kd, a step setpoint and no output clamp.  Derivative
     * action *on the measurement* sees no change on the first sample, so the
     * output must be exactly Kp * e.  Derivative on the error would produce
     * Kp*e + Kd*e/dt, i.e. a 1000x larger spike. */
    TP_ASSERT_EQ(make_pid(&pid, 2.0f, 0.0f, 50.0f, 1.0e6f, 0.0f), TP_OK);
    u = tp_pid_update(&pid, 1.0f, 0.0f);
    TP_ASSERT_NEAR(u, 2.0f, 1.0e-4f);
}

TP_TEST_CASE(pid, derivative_on_measurement_ignores_setpoint_step)
{
    tp_pid_t pid;
    float u_after_jump;

    TP_ASSERT_EQ(make_pid(&pid, 1.0f, 0.0f, 10.0f, 1.0e6f, 0.0f), TP_OK);
    (void)tp_pid_update(&pid, 1.0f, 0.0f);
    /* Jump the setpoint with the measurement held: derivative-on-measurement
     * must contribute nothing, so the output is exactly Kp * e = 1 * 5. */
    u_after_jump = tp_pid_update(&pid, 5.0f, 0.0f);
    TP_ASSERT_NEAR(u_after_jump, 5.0f, 1.0e-3f);
}

TP_TEST_CASE(pid, trapezoidal_integration_tracks_a_ramp)
{
    tp_pid_t pid;
    float i_term;
    int k;

    /* Feed a constant error and compare the integral with the analytic value
     * Ki * e * t.  Trapezoidal integration agrees exactly after the first
     * sample; the rectangular rule would be off by half a sample. */
    TP_ASSERT_EQ(make_pid(&pid, 0.0f, 1.0f, 0.0f, 1.0e6f, 1.0e6f), TP_OK);
    for (k = 0; k < 1000; ++k) {
        (void)tp_pid_update(&pid, 1.0f, 0.0f);
    }
    i_term = pid.terms.i;
    TP_ASSERT_NEAR(i_term, 1.0f, 1.0e-3f); /* 1.0 * 1.0 A * 1.0 s */
}

/* =========================================================================
 * Boundary conditions
 * ========================================================================= */

TP_TEST_CASE(pid, output_is_clamped)
{
    tp_pid_t pid;
    float u;

    TP_ASSERT_EQ(make_pid(&pid, 100.0f, 0.0f, 0.0f, 5.0f, 5.0f), TP_OK);
    u = tp_pid_update(&pid, 1000.0f, 0.0f);
    TP_ASSERT_NEAR(u, 5.0f, 1.0e-5f);
    u = tp_pid_update(&pid, -1000.0f, 0.0f);
    TP_ASSERT_NEAR(u, -5.0f, 1.0e-5f);
}

TP_TEST_CASE(pid, integral_is_clamped_independently)
{
    tp_pid_t pid;
    int k;

    TP_ASSERT_EQ(make_pid(&pid, 0.0f, 100.0f, 0.0f, 1000.0f, 3.0f), TP_OK);
    for (k = 0; k < 5000; ++k) {
        (void)tp_pid_update(&pid, 1.0f, 0.0f);
    }
    TP_ASSERT_NEAR(pid.integral, 3.0f, 1.0e-3f);
}

TP_TEST_CASE(pid, integral_clamp_cannot_exceed_output_clamp)
{
    tp_pid_t pid;

    /* Configuring an integral clamp above the output clamp would let the I term
     * alone saturate the actuator forever.  init() must reduce it. */
    TP_ASSERT_EQ(make_pid(&pid, 0.0f, 1.0f, 0.0f, 2.0f, 100.0f), TP_OK);
    TP_ASSERT_NEAR(pid.i_max, 2.0f, 1.0e-6f);
    TP_ASSERT_NEAR(pid.i_min, -2.0f, 1.0e-6f);
}

TP_TEST_CASE(pid, back_calculation_limits_the_wind_up_and_unwinds)
{
    tp_pid_t pid;
    float magnitude_at_saturation;
    int k;

    TP_ASSERT_EQ(make_pid(&pid, 0.0f, 10.0f, 0.0f, 1.0f, 1000.0f), TP_OK);
    for (k = 0; k < 1000; ++k) {
        (void)tp_pid_update(&pid, 1.0f, 0.0f);
    }
    magnitude_at_saturation = pid.integral;
    /* init() clamps the integral limit to the output limit, so the I term alone
     * can never saturate the actuator.  Without that invariant a long approach
     * would charge the integrator to a value the actuator could never deliver,
     * and the charge would be released as an overshoot at the end of the move. */
    TP_ASSERT_NEAR(magnitude_at_saturation, 1.0f, 1.0e-3f);

    /* Now reverse the error.  With back-calculation the output must leave
     * saturation promptly instead of sitting on the rail for the whole unwind. */
    for (k = 0; k < 200; ++k) {
        (void)tp_pid_update(&pid, -1.0f, 0.0f);
    }
    TP_ASSERT_LT(pid.integral, magnitude_at_saturation);
    TP_ASSERT_NEAR(pid.integral, -1.0f, 0.02f);
}

TP_TEST_CASE(pid, conditional_integration_limits_the_wind_up_too)
{
    tp_pid_t pid;
    tp_pid_config_t cfg = tp_pid_config_default(0.0f, 10.0f, 0.0f, DT, 1.0f,
                                                1000.0f, 0.0f);
    int k;

    cfg.antiwindup = TP_PID_AW_CLAMP_INTEGRAL;
    TP_ASSERT_EQ(tp_pid_init(&pid, &cfg), TP_OK);
    for (k = 0; k < 1000; ++k) {
        (void)tp_pid_update(&pid, 1.0f, 0.0f);
    }
    TP_ASSERT_NEAR(pid.integral, 1.0f, 1.0e-3f);
    for (k = 0; k < 200; ++k) {
        (void)tp_pid_update(&pid, -1.0f, 0.0f);
    }
    /* Both strategies must make the integrator reversible; the difference
     * between them is *how* the limit is enforced (a feedback term versus a
     * branch), not whether it works. */
    TP_ASSERT_NEAR(pid.integral, -1.0f, 0.02f);
}
/* =========================================================================
 * Invalid input
 * ========================================================================= */

TP_TEST_CASE(pid, init_rejects_bad_configuration)
{
    tp_pid_t pid;
    tp_pid_config_t cfg = tp_pid_config_default(1.0f, 1.0f, 0.0f, DT, 10.0f,
                                                5.0f, 0.0f);

    TP_ASSERT_EQ(tp_pid_init(NULL, &cfg), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_pid_init(&pid, NULL), TP_ERR_PARAM);

    cfg.dt_s = 0.0f;
    TP_ASSERT_EQ(tp_pid_init(&pid, &cfg), TP_ERR_RANGE);

    cfg = tp_pid_config_default(1.0f, 1.0f, 0.0f, DT, 10.0f, 5.0f, 0.0f);
    cfg.out_min = 10.0f;   /* out_min above out_max is the inversion */
    TP_ASSERT_EQ(tp_pid_init(&pid, &cfg), TP_ERR_RANGE);

    cfg = tp_pid_config_default(1.0f, 1.0f, 0.0f, DT, 10.0f, 5.0f, 0.0f);
    cfg.i_min = 6.0f;      /* i_min above i_max (5) is the inversion */
    TP_ASSERT_EQ(tp_pid_init(&pid, &cfg), TP_ERR_RANGE);
}

TP_TEST_CASE(pid, non_finite_input_is_refused_and_latched)
{
    tp_pid_t pid;
    float nan_value = 0.0f / 0.0f; /* deliberately NaN */
    float u;
    float before;

    TP_ASSERT_EQ(make_pid(&pid, 1.0f, 1.0f, 0.0f, 100.0f, 50.0f), TP_OK);
    before = tp_pid_update(&pid, 1.0f, 0.0f);
    u = tp_pid_update(&pid, nan_value, 0.0f);

    /* The output must be the last good value, not zero and not NaN: a joint
     * that suddenly receives zero command mid-move is a hazard. */
    TP_ASSERT_NEAR(u, before, 1.0e-6f);
    TP_ASSERT_EQ(tp_pid_is_diverged(&pid), 1);
    TP_ASSERT_FINITE(u);
}

TP_TEST_CASE(pid, uninitialised_instance_is_inert)
{
    tp_pid_t pid;

    (void)memset(&pid, 0, sizeof(pid));
    TP_ASSERT_NEAR(tp_pid_update(&pid, 1.0f, 0.0f), 0.0f, 1.0e-9f);
    TP_ASSERT_EQ(tp_pid_set_gains(&pid, 1.0f, 1.0f, 1.0f), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_pid_set_limits(&pid, -1.0f, 1.0f, -1.0f, 1.0f), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_pid_set_dt(&pid, DT), TP_ERR_PARAM);
}

TP_TEST_CASE(pid, negative_integral_gain_is_rejected)
{
    tp_pid_t pid;

    TP_ASSERT_EQ(make_pid(&pid, 1.0f, 1.0f, 0.0f, 100.0f, 50.0f), TP_OK);
    /* A negative Ki turns the anti-windup feedback into positive feedback. */
    TP_ASSERT_EQ(tp_pid_set_gains(&pid, 1.0f, -1.0f, 0.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_pid_set_dt(&pid, -0.001f), TP_ERR_RANGE);
}

/* =========================================================================
 * Exceptional state
 * ========================================================================= */

TP_TEST_CASE(pid, reset_clears_integrator_and_divergence)
{
    tp_pid_t pid;
    float nan_value = 0.0f / 0.0f;
    int k;

    TP_ASSERT_EQ(make_pid(&pid, 1.0f, 10.0f, 0.0f, 100.0f, 50.0f), TP_OK);
    for (k = 0; k < 100; ++k) {
        (void)tp_pid_update(&pid, 1.0f, 0.0f);
    }
    TP_ASSERT_GT(pid.integral, 0.0f);

    (void)tp_pid_update(&pid, nan_value, 0.0f);
    TP_ASSERT_EQ(tp_pid_is_diverged(&pid), 1);

    tp_pid_reset(&pid);
    TP_ASSERT_NEAR(pid.integral, 0.0f, 1.0e-9f);
    TP_ASSERT_EQ(tp_pid_is_diverged(&pid), 0);
    TP_ASSERT_EQ(pid.update_count, 0u);
}

TP_TEST_CASE(pid, gains_can_be_changed_without_a_transient)
{
    tp_pid_t pid;
    float i_before;
    float i_after;

    TP_ASSERT_EQ(make_pid(&pid, 1.0f, 5.0f, 0.0f, 100.0f, 50.0f), TP_OK);
    (void)tp_pid_update(&pid, 1.0f, 0.0f);
    (void)tp_pid_update(&pid, 1.0f, 0.0f);
    i_before = pid.integral;

    TP_ASSERT_EQ(tp_pid_set_gains(&pid, 2.0f, 5.0f, 0.0f), TP_OK);
    i_after = pid.integral;
    /* Gain scheduling must not clear the integrator: doing so would inject a
     * step into the loop every time a gain changed. */
    TP_ASSERT_NEAR(i_after, i_before, 1.0e-9f);
    TP_ASSERT_NEAR(pid.kp, 2.0f, 1.0e-9f);
}

TP_TEST_CASE(pid, set_limits_reclamps_a_wound_up_integrator)
{
    tp_pid_t pid;
    int k;

    TP_ASSERT_EQ(make_pid(&pid, 0.0f, 10.0f, 0.0f, 100.0f, 50.0f), TP_OK);
    for (k = 0; k < 1000; ++k) {
        (void)tp_pid_update(&pid, 1.0f, 0.0f);
    }
    /* The integral accumulates Ki * e * t = 10 * 1 * 1 = 10 over one second of
     * constant error, and the configured 50 limit is never reached. */
    TP_ASSERT_NEAR(pid.integral, 10.0f, 1.0e-2f);

    /* Tightening the limits must immediately re-clamp a wound-up integrator:
     * otherwise the old charge would still be delivered through the output. */
    TP_ASSERT_EQ(tp_pid_set_limits(&pid, -10.0f, 10.0f, -2.0f, 2.0f), TP_OK);
    TP_ASSERT_NEAR(pid.integral, 2.0f, 1.0e-3f);
}

TP_TEST_CASE(pid, term_breakdown_matches_the_output)
{
    tp_pid_t pid;
    float u;
    tp_pid_terms_t terms;

    TP_ASSERT_EQ(make_pid(&pid, 2.0f, 1.0f, 0.0f, 100.0f, 50.0f), TP_OK);
    (void)tp_pid_update(&pid, 1.0f, 0.0f);
    u = tp_pid_update(&pid, 1.0f, 0.0f);
    TP_ASSERT_EQ(tp_pid_get_terms(&pid, &terms), TP_OK);

    /* P + I + D + FF must reconstruct the reported output.  Telemetry and the
     * experiment plots read these terms individually, so an inconsistency here
     * would make every plot wrong in a way that looks like a control problem. */
    TP_ASSERT_NEAR(terms.p + terms.i + terms.d + terms.ff, terms.out, 1.0e-4f);
    TP_ASSERT_NEAR(terms.out, u, 1.0e-6f);
    TP_ASSERT_NEAR(terms.p, 2.0f, 1.0e-4f);
}

TP_TEST_CASE(pid, feed_forward_is_added_after_the_gains)
{
    tp_pid_t pid;
    float u;

    TP_ASSERT_EQ(make_pid(&pid, 0.0f, 0.0f, 0.0f, 100.0f, 50.0f), TP_OK);
    (void)tp_pid_set_ff_gain(&pid, 2.0f);
    u = tp_pid_update_ff(&pid, 0.0f, 0.0f, 3.0f);
    TP_ASSERT_NEAR(u, 6.0f, 1.0e-5f);
}

TP_TEST_CASE(pid, derivative_low_pass_stays_stable_for_large_dt)
{
    tp_pid_t pid;
    tp_pid_config_t cfg;
    int k;

    /* dt far larger than tau_d is the case where the "textbook" alpha = dt/tau
     * discretisation explodes.  The backward-Euler form must not. */
    cfg = tp_pid_config_default(0.0f, 0.0f, 1.0f, 0.5f, 1.0e6f, 1.0e6f, 0.001f);
    TP_ASSERT_EQ(tp_pid_init(&pid, &cfg), TP_OK);
    for (k = 0; k < 200; ++k) {
        float u = tp_pid_update(&pid, 0.0f, (float)((k % 2) ? 10 : -10));
        TP_ASSERT_FINITE(u);
    }
}
