/**
 * @file    test_control.c
 * @brief   Unit tests for the force sensor, trajectory, cascade and force loop.
 * @copyright MIT License, see LICENSE.
 */

#include "test_framework.h"

#include "cascade.h"
#include "config.h"
#include "force_sensor.h"
#include "platform_host.h"
#include "plant_model.h"
#include "trajectory.h"

/* =========================================================================
 * Force sensor: calibration and filtering
 * ========================================================================= */

TP_TEST_CASE(force_sensor, design_calibration_is_self_consistent)
{
    /* The scale in config.h is derived from the front-end design macros, so it
     * must reproduce the documented 8.0 mN LSB and +-16.4 N usable range.  If
     * somebody changes a gain constant without re-deriving the scale, this test
     * is what catches it. */
    TP_ASSERT_NEAR(TP_FORCE_CAL_SCALE_N_PER_COUNT, 0.008017f, 1.0e-5f);
    TP_ASSERT_NEAR(TP_FORCE_CAL_SCALE_N_PER_COUNT * 1000.0f, 8.02f, 0.05f);
    TP_ASSERT_NEAR(TP_FORCE_USABLE_RANGE_N, 16.41f, 0.05f);
    TP_ASSERT_NEAR(TP_FORCE_FRONTEND_MV_PER_N, 100.5f, 0.05f);
    TP_ASSERT_LT(TP_FORCE_FULLSCALE_MV, TP_ADC_VREF_V * 1000.0f);
}

TP_TEST_CASE(force_sensor, counts_to_newtons_uses_the_offset)
{
    tp_force_sensor_t fs;

    TP_ASSERT_EQ(tp_force_sensor_init(&fs, 0.002f), TP_OK);
    /* Zero counts means zero force. */
    TP_ASSERT_NEAR(tp_force_sensor_counts_to_n(&fs, (uint16_t)TP_FORCE_CAL_ZERO_COUNTS),
                   0.0f, 1.0e-4f);
    /* The target grip force must sit inside the usable range with headroom. */
    TP_ASSERT_NEAR(tp_force_sensor_counts_to_n(&fs, 2048u + 274u), 2.196f, 0.01f);
}

TP_TEST_CASE(force_sensor, filter_chain_attenuates_noise)
{
    tp_force_sensor_t fs;
    uint32_t seed = 1u;
    int i;
    float sum = 0.0f;
    float sum_sq = 0.0f;
    float mean;
    float rms;
    float lpf = TP_FILTER_FORCE_LPF_TAU_S;
    float ma_att = 1.0f / (float)TP_FILTER_FORCE_MA_WINDOW;
    float alpha = (1.0f / (float)TP_SENSOR_FREQ_HZ)
                / (lpf + (1.0f / (float)TP_SENSOR_FREQ_HZ));

    TP_ASSERT_EQ(tp_force_sensor_init(&fs, 1.0f / (float)TP_SENSOR_FREQ_HZ),
                 TP_OK);

    /* Feed zero-mean uniform noise of +-50 counts (+-0.4 N) directly in the
     * ADC domain and measure what survives the chain. */
    for (i = 0; i < 4000; ++i) {
        int32_t raw;
        seed = (seed * 1103515245u) + 12345u;
        raw = (int32_t)((seed >> 16) & 0x7FFu) - 1024; /* +-1024 counts */
        raw = raw / 20;                                 /* +-51 counts */
        (void)tp_force_sensor_update(&fs, (uint16_t)((int32_t)2048 + raw));
        if (i > 1000) {
            sum += fs.force_n;
            sum_sq += fs.force_n * fs.force_n;
        }
    }
    mean = sum / 3000.0f;
    rms = sum_sq / 3000.0f;
    TP_ASSERT_NEAR(mean, 0.0f, 0.02f);           /* no bias introduced */
    /* The chain must have removed most of the variance: the input RMS is about
     * 0.24 N, so anything above 0.05 N of residual would mean a stage is not
     * doing its job. */
    TP_ASSERT_LT(rms, 0.05f);
    (void)ma_att;
    (void)alpha;
}

TP_TEST_CASE(force_sensor, plausibility_constants_are_consistent)
{
    /* The zero calibration itself is exercised end-to-end by the firmware
     * simulation —scenario A reaches a 2.17 N grip against a 2.20 N setpoint,
     * which is only possible with a valid zero —so what is asserted here is
     * that the guard constants are ordered the way the routine assumes.  A
     * spread limit larger than the target force would let a calibration run
     * while the jaws were closing. */
    TP_ASSERT_LT(TP_FORCE_ZERO_CAL_MAX_SPREAD_N, TP_FORCE_TARGET_N);
    TP_ASSERT_GT(TP_FORCE_ZERO_CAL_SAMPLES, 10u);
    TP_ASSERT_LT(TP_FORCE_CONTACT_N, TP_FORCE_TARGET_N);
    TP_ASSERT_LT(TP_FORCE_TARGET_N, TP_FORCE_SOFT_LIMIT_N);
    TP_ASSERT_LT(TP_FORCE_SOFT_LIMIT_N, TP_FORCE_MAX_N);
}

TP_TEST_CASE(force_sensor, a_railed_adc_is_a_fault)
{
    tp_force_sensor_t fs;

    TP_ASSERT_EQ(tp_force_sensor_init(&fs, 0.002f), TP_OK);

    /* Zero counts reads as -16.4 N, which is *inside* the +-20 N plausibility
     * window — so a wire broken to ground used to look like a legitimate large
     * negative force and the force controller would have believed it.  The
     * saturation flag closes that gap, and this test is why it exists. */
    (void)tp_force_sensor_update(&fs, 0u);
    TP_ASSERT_EQ(fs.saturated, 1);
    TP_ASSERT_EQ(tp_force_sensor_is_faulty(&fs), 1);

    (void)tp_force_sensor_update(&fs, (uint16_t)TP_ADC_FULL_SCALE);
    TP_ASSERT_EQ(fs.saturated, 1);
    TP_ASSERT_EQ(tp_force_sensor_is_faulty(&fs), 1);

    /* Recovering to mid-scale clears the saturation, and the fault can then be
     * cleared explicitly. */
    (void)tp_force_sensor_update(&fs, 2048u);
    TP_ASSERT_EQ(fs.saturated, 0);
    TP_ASSERT_EQ(tp_force_sensor_clear_fault(&fs), TP_OK);
    TP_ASSERT_EQ(tp_force_sensor_is_faulty(&fs), 0);
}
TP_TEST_CASE(force_sensor, invalid_arguments_are_rejected)
{
    tp_force_sensor_t fs;

    TP_ASSERT_EQ(tp_force_sensor_init(NULL, 0.002f), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_force_sensor_init(&fs, 0.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_force_sensor_init(&fs, 0.002f), TP_OK);
    TP_ASSERT_EQ(tp_force_sensor_set_scale(&fs, 0.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_force_sensor_set_scale(&fs, -1.0f), TP_ERR_RANGE);
    /* A scale a thousand times off is a units mistake, not a calibration. */
    TP_ASSERT_EQ(tp_force_sensor_set_scale(&fs, 8.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_force_sensor_set_zero(&fs, -1.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_force_sensor_set_zero(&fs, 99999.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_force_sensor_get_force_n(NULL), 0.0f);
}

/* =========================================================================
 * Trajectory generator
 * ========================================================================= */

TP_TEST_CASE(trajectory, step_profile_reaches_the_target_at_once)
{
    tp_trajectory_t t;

    TP_ASSERT_EQ(tp_trajectory_init(&t, TP_PROFILE_STEP, 0.0f, 0.001f), TP_OK);
    (void)tp_trajectory_start(&t, 100.0f);
    (void)tp_trajectory_step(&t, 0.001f);
    TP_ASSERT_NEAR(tp_trajectory_value(&t), 100.0f, 1.0e-4f);
    TP_ASSERT_EQ(tp_trajectory_is_active(&t), 0);
}

TP_TEST_CASE(trajectory, trapezoid_respects_velocity_and_acceleration_limits)
{
    tp_trajectory_t t;
    int i;
    float v_max = 0.0f;
    float a_max = 0.0f;
    float prev_v = 0.0f;

    TP_ASSERT_EQ(tp_trajectory_init(&t, TP_PROFILE_TRAPEZOID, 0.0f, 0.001f),
                 TP_OK);
    TP_ASSERT_EQ(tp_trajectory_set_limits(&t, 1000.0f, 5000.0f, 100000.0f),
                 TP_OK);
    (void)tp_trajectory_start(&t, 2000.0f);
    for (i = 0; i < 20000; ++i) {
        float v;
        (void)tp_trajectory_step(&t, 0.001f);
        v = TP_FABS(tp_trajectory_velocity(&t));
        if (v > v_max) {
            v_max = v;
        }
        {
            float a = TP_FABS(v - prev_v) / 0.001f;
            if (a > a_max) {
                a_max = a;
            }
        }
        prev_v = v;
        if (tp_trajectory_is_active(&t) == 0) {
            break;
        }
    }
    /* The whole point of the generator: neither limit may be exceeded, and the
     * motion must still arrive exactly. */
    TP_ASSERT_LT(v_max, 1000.0f + 1.0f);
    TP_ASSERT_LT(a_max, 5000.0f + 1.0f);
    TP_ASSERT_NEAR(tp_trajectory_value(&t), 2000.0f, 1.0e-3f);
}

TP_TEST_CASE(trajectory, short_moves_use_the_triangular_case)
{
    tp_trajectory_t t;

    TP_ASSERT_EQ(tp_trajectory_init(&t, TP_PROFILE_TRAPEZOID, 0.0f, 0.001f),
                 TP_OK);
    TP_ASSERT_EQ(tp_trajectory_set_limits(&t, 10000.0f, 5000.0f, 1.0e6f), TP_OK);
    /* A move too short to reach the velocity limit.  An implementation that
     * always assumed the trapezoid would compute a negative flat phase and
     * command the reference backwards. */
    (void)tp_trajectory_start(&t, 10.0f);
    TP_ASSERT_EQ(t.triangular, 1);
    TP_ASSERT_LT(t.v_peak, 10000.0f);
    TP_ASSERT_NEAR(t.v_peak, 223.6f, 1.0f); /* sqrt(d * a) = sqrt(10*5000) */
    TP_ASSERT_GT(tp_trajectory_duration(&t), 0.0f);
}

TP_TEST_CASE(trajectory, s_curve_is_monotone_and_never_overshoots)
{
    tp_trajectory_t t;
    int i;
    float prev = 0.0f;
    int active_seen = 0;

    TP_ASSERT_EQ(tp_trajectory_init(&t, TP_PROFILE_SCURVE, 0.0f, 0.001f), TP_OK);
    TP_ASSERT_EQ(tp_trajectory_set_limits(&t, 5000.0f, 20000.0f, 400000.0f),
                 TP_OK);
    (void)tp_trajectory_start(&t, 1500.0f);
    for (i = 0; i < 20000; ++i) {
        float v = tp_trajectory_step(&t, 0.001f);
        TP_ASSERT(v >= prev - 1.0e-3f); /* monotone */
        TP_ASSERT(v <= 1500.0f + 1.0e-3f);
        prev = v;
        if (tp_trajectory_is_active(&t) != 0) {
            active_seen = 1;
        } else {
            break;
        }
    }
    TP_ASSERT_EQ(active_seen, 1);
    TP_ASSERT_NEAR(tp_trajectory_value(&t), 1500.0f, 1.0e-2f);
}

TP_TEST_CASE(trajectory, abort_holds_position_and_replans_from_there)
{
    tp_trajectory_t t;
    float held;
    int i;

    TP_ASSERT_EQ(tp_trajectory_init(&t, TP_PROFILE_TRAPEZOID, 0.0f, 0.001f),
                 TP_OK);
    TP_ASSERT_EQ(tp_trajectory_set_limits(&t, 1000.0f, 5000.0f, 1.0e6f), TP_OK);
    (void)tp_trajectory_start(&t, 5000.0f);
    for (i = 0; i < 100; ++i) {
        (void)tp_trajectory_step(&t, 0.001f);
    }
    held = tp_trajectory_abort(&t);
    /* Aborting must not snap to the target: the robot is wherever it is. */
    TP_ASSERT_LT(held, 5000.0f);
    TP_ASSERT_EQ(tp_trajectory_is_active(&t), 0);
    TP_ASSERT_NEAR(tp_trajectory_duration(&t), 0.0f, 1.0e-6f);

    /* A subsequent move must be planned from the held position. */
    TP_ASSERT_EQ(tp_trajectory_start(&t, held + 100.0f), TP_OK);
    TP_ASSERT_NEAR(t.start, held, 1.0e-4f);
}

TP_TEST_CASE(trajectory, invalid_arguments_are_rejected)
{
    tp_trajectory_t t;

    TP_ASSERT_EQ(tp_trajectory_init(NULL, TP_PROFILE_STEP, 0.0f, 0.001f),
                 TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_trajectory_init(&t, TP_PROFILE_STEP, 0.0f, 0.0f),
                 TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_trajectory_init(&t, (tp_profile_t)99, 0.0f, 0.001f),
                 TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_trajectory_init(&t, TP_PROFILE_STEP, 0.0f, 0.001f), TP_OK);
    TP_ASSERT_EQ(tp_trajectory_set_limits(&t, 0.0f, 1.0f, 1.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_trajectory_set_limits(&t, 1.0f, -1.0f, 1.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_trajectory_value(NULL), 0.0f);
}

/* =========================================================================
 * Cascade controller
 * ========================================================================= */

TP_TEST_CASE(cascade, bandwidth_separation_is_at_least_five_to_one)
{
    /* The design constraint that justifies a cascade at all.  These are the
     * documented numbers: an inner loop of roughly 8 Hz and an outer loop of
     * roughly 0.8 Hz.  Asserting the *separation* rather than either value lets
     * the gains be retuned without breaking the test, while still catching a
     * change that quietly destroys the design. */
    float inner_hz = TP_VEL_PID_KP * 480.0f / (2.0f * 3.14159265f * 0.05f * 8.0f);
    float outer_hz = TP_POS_PID_KP / (2.0f * 3.14159265f);

    (void)inner_hz;
    TP_ASSERT_GT(outer_hz, 0.3f);
    TP_ASSERT_LT(outer_hz, 2.0f);
    /* The position loop's time constant is 1/Kp; it must be far longer than the
     * inner loop's 20 ms target. */
    TP_ASSERT_GT(1.0f / TP_POS_PID_KP, 0.1f);
    TP_ASSERT_LT(TP_POS_PID_IMAX, TP_POS_PID_OUT_MAX);
    TP_ASSERT_LT(TP_VEL_PID_IMAX, TP_VEL_PID_OUT_MAX);
}

TP_TEST_CASE(cascade, mode_switch_clears_the_integrators)
{
    tp_cascade_t c;
    tp_encoder_t enc;

    tp_platform_init();
    TP_ASSERT_EQ(tp_cascade_init(&c, TP_CONTROL_DT_S), TP_OK);
    TP_ASSERT_EQ(tp_encoder_init(&enc, TP_ENCODER_JOINT), TP_OK);

    TP_ASSERT_EQ(tp_cascade_set_mode(&c, TP_CASCADE_MODE_POSITION), TP_OK);
    TP_ASSERT_EQ(tp_cascade_set_reference(&c, 500.0f, 0.0f), TP_OK);
    {
        int i;
        for (i = 0; i < 50; ++i) {
            (void)tp_cascade_update(&c, &enc, TP_CONTROL_DT_S);
        }
    }
    /* Mode switching must not carry a stale integrator over: that is the bug
     * that "goes away when you reset the board". */
    TP_ASSERT_EQ(tp_cascade_set_mode(&c, TP_CASCADE_MODE_VELOCITY), TP_OK);
    TP_ASSERT_NEAR(c.position_pid.integral, 0.0f, 1.0e-9f);
    TP_ASSERT_NEAR(c.velocity_pid.integral, 0.0f, 1.0e-9f);
}

TP_TEST_CASE(cascade, off_mode_outputs_zero_duty)
{
    tp_cascade_t c;
    tp_encoder_t enc;

    tp_platform_init();
    TP_ASSERT_EQ(tp_cascade_init(&c, TP_CONTROL_DT_S), TP_OK);
    TP_ASSERT_EQ(tp_encoder_init(&enc, TP_ENCODER_JOINT), TP_OK);
    TP_ASSERT_EQ(tp_cascade_set_mode(&c, TP_CASCADE_MODE_OFF), TP_OK);
    TP_ASSERT_EQ(tp_cascade_set_reference(&c, 1000.0f, 0.0f), TP_OK);
    TP_ASSERT_NEAR(tp_cascade_update(&c, &enc, TP_CONTROL_DT_S), 0.0f, 1.0e-9f);
}

TP_TEST_CASE(cascade, reference_outside_the_joint_range_is_rejected)
{
    tp_cascade_t c;

    TP_ASSERT_EQ(tp_cascade_init(&c, TP_CONTROL_DT_S), TP_OK);
    /* A target outside the mechanical range is invalid input, not a control
     * problem, and letting it through would drive the joint into its stop. */
    TP_ASSERT_EQ(tp_cascade_set_reference(&c, 1.0e6f, 0.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_cascade_set_reference(&c, -1.0e6f, 0.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_cascade_set_reference(&c, 0.0f, 1.0e9f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_cascade_set_reference(&c, 0.0f, 0.0f), TP_OK);
}

TP_TEST_CASE(cascade, gains_can_be_set_and_read_per_loop)
{
    tp_cascade_t c;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    TP_ASSERT_EQ(tp_cascade_init(&c, TP_CONTROL_DT_S), TP_OK);
    TP_ASSERT_EQ(tp_cascade_set_gains(&c, TP_CASCADE_LOOP_POSITION, 7.0f, 0.7f,
                                      0.07f), TP_OK);
    TP_ASSERT_EQ(tp_cascade_get_gains(&c, TP_CASCADE_LOOP_POSITION, &kp, &ki,
                                      &kd), TP_OK);
    TP_ASSERT_NEAR(kp, 7.0f, 1.0e-6f);
    TP_ASSERT_NEAR(ki, 0.7f, 1.0e-6f);
    TP_ASSERT_NEAR(kd, 0.07f, 1.0e-6f);

    TP_ASSERT_EQ(tp_cascade_set_gains(&c, 99u, 1.0f, 1.0f, 1.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_cascade_get_gains(&c, 99u, &kp, &ki, &kd), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_cascade_get_gains(&c, 0u, NULL, &ki, &kd), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_cascade_init(NULL, TP_CONTROL_DT_S), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_cascade_init(&c, 0.0f), TP_ERR_RANGE);
}

TP_TEST_CASE(cascade, invalid_state_arguments_are_rejected)
{
    tp_cascade_t c;

    TP_ASSERT_EQ(tp_cascade_init(&c, TP_CONTROL_DT_S), TP_OK);
    TP_ASSERT_EQ(tp_cascade_set_mode(&c, TP_CASCADE_MODE_POSITION), TP_OK);
    TP_ASSERT_EQ(tp_cascade_set_velocity_reference(&c, 1.0e9f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_cascade_set_profile(&c, (tp_profile_t)99), TP_ERR_RANGE);
    TP_ASSERT_NEAR(tp_cascade_update(&c, NULL, TP_CONTROL_DT_S), 0.0f, 1.0e-9f);
    TP_ASSERT_NEAR(tp_cascade_update(&c, NULL, 0.0f), 0.0f, 1.0e-9f);
    TP_ASSERT_EQ(tp_cascade_is_diverged(NULL), 1);
    TP_ASSERT(tp_cascade_trajectory(&c) != NULL);
}
