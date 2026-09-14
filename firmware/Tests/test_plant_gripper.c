/**
 * @file    test_plant_gripper.c
 * @brief   Plant-model and gripper-driver tests, including the sign-convention
 *          cross-check between the two.
 * @copyright MIT License, see LICENSE.
 */

#include "test_framework.h"

#include "config.h"
#include "encoder.h"
#include "gripper.h"
#include "limit_switch.h"
#include "motor.h"
#include "platform_host.h"
#include "plant_model.h"

/** @brief Integration step used by the plant tests. */
#define PSTEP 0.001f

/**
 * @brief Advance the plant and the platform clock together.
 *
 * @param[in] steps  Number of 1 ms steps.
 */
static void plant_run(int steps)
{
    int i;
    for (i = 0; i < steps; ++i) {
        tp_host_platform_advance(1000u);
    }
}

/* =========================================================================
 * Sign conventions —the cross-check the design promised
 * ========================================================================= */

TP_TEST_CASE(plant_gripper, duty_sign_agrees_between_driver_and_plant)
{
    /*
     * `gripper.c` maps a jaw velocity to a duty with its own arithmetic; the
     * plant maps a duty back to a velocity.  They are two independent
     * expressions of one convention and they *did* disagree in the first
     * revision, which made the gripper open when commanded to close and produced
     * a "no contact within timeout" fault that looked like a controller problem.
     *
     * This test is the reason the duplication is acceptable: the two mappings
     * are asserted to be inverses, so they cannot drift apart again.
     */
    float v;
    for (v = -100.0f; v <= 100.0f; v += 25.0f) {
        float duty = tp_plant_gripper_velocity_to_pwm(v);
        /* Positive duty must close, and closing is a negative velocity.  So a
         * negative v must give a positive duty, and vice versa. */
        if (v < -0.01f) {
            TP_ASSERT_MSG(duty > 0.0f, "closing must command positive duty");
        } else if (v > 0.01f) {
            TP_ASSERT_MSG(duty < 0.0f, "opening must command negative duty");
        }
    }
    TP_ASSERT_NEAR(tp_plant_gripper_velocity_to_pwm(-TP_GRIPPER_MAX_SPEED_MM_S),
                   TP_MAX_PWM_PERMILLE, 1.0e-3f);
    TP_ASSERT_NEAR(tp_plant_gripper_velocity_to_pwm(TP_GRIPPER_MAX_SPEED_MM_S),
                   -TP_MAX_PWM_PERMILLE, 1.0e-3f);
    TP_ASSERT_NEAR(tp_plant_gripper_velocity_to_pwm(300.0f),
                   -TP_MAX_PWM_PERMILLE, 1.0e-3f); /* saturated */
}

TP_TEST_CASE(plant_gripper, positive_duty_closes_the_jaw)
{
    tp_plant_t p;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    TP_ASSERT_NEAR(p.gripper.gap_mm, TP_GRIPPER_MAX_MM, 1.0e-4f);

    /* 200 ms of positive duty must reduce the gap. */
    {
        int i;
        for (i = 0; i < 200; ++i) {
            tp_plant_step(&p, 0.0f, 500.0f, PSTEP);
        }
    }
    TP_ASSERT_LT(p.gripper.gap_mm, TP_GRIPPER_MAX_MM);
    TP_ASSERT_FINITE(p.gripper.gap_mm);
}

TP_TEST_CASE(plant_gripper, contact_force_is_nonlinear_and_monotone)
{
    tp_plant_t p;
    float f1;
    float f2;
    float f3;
    int i;

    /*
     * The jaw starts fully open at 40 mm, closes at 36 mm/s (300 per-mille
     * scaled by the 120 mm/s rating) and meets the stem surface at 6 mm, which
     * is about 944 steps away; the mechanical stop at 1 mm is about 1083 steps
     * away.  Every sample below therefore has to fall inside that 140-step
     * window — running all three for thousands of steps puts all three hard
     * against the stop, where they agree at 3.0 N and the test proves nothing.
     * That is exactly how the first version of this test passed 4000 steps and
     * still measured three identical forces.
     *
     * The three sample points are spaced by *equal* step counts on purpose.
     * They used to be 1010 / 1050 / 1082 — windows of 40 and 32 steps — so the
     * comparison mixed curvature with window length and passed only while the
     * curvature happened to win.  Correcting the force back-drive sign (see
     * `plant_model.c`) tipped that balance and the test failed with 0.868
     * against 0.884: the smaller increment belonged to the shorter window, not
     * to a change in the physics.  Equal windows measure what the assertion
     * claims to measure.
     */
    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    tp_plant_set_stem_gap(&p, 6.0f);
    for (i = 0; i < 1010; ++i) {
        tp_plant_step(&p, 0.0f, 300.0f, PSTEP);
    }
    f1 = p.gripper.force_n;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    tp_plant_set_stem_gap(&p, 6.0f);
    for (i = 0; i < 1042; ++i) {
        tp_plant_step(&p, 0.0f, 300.0f, PSTEP);
    }
    f2 = p.gripper.force_n;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    tp_plant_set_stem_gap(&p, 6.0f);
    for (i = 0; i < 1074; ++i) {
        tp_plant_step(&p, 0.0f, 300.0f, PSTEP);
    }
    f3 = p.gripper.force_n;

    TP_ASSERT_MSG(f1 > 0.0f, "no contact yet at 1010 steps");
    TP_ASSERT_MSG(f2 > f1, "force must rise with penetration");
    TP_ASSERT_MSG(f3 > f2, "force must keep rising with penetration");
    TP_ASSERT_FINITE(f3);

    /* And the stiffening must be super-linear: the quadratic term is the whole
     * reason a fixed-gap gripper is unsafe, so the force gained over the second
     * half of the travel must exceed the force gained over the first. */
    TP_ASSERT_GT(f3 - f2, f2 - f1);
}
TP_TEST_CASE(plant_gripper, a_free_jaw_is_pushed_open_by_the_stem)
{
    /*
     * The force back-drive term is the only place the stem's reaction reaches
     * the jaw's own dynamics, and its sign decides whether a free jaw is stable
     * or not.
     *
     * Squeeze the stem, then command zero duty and let the plant run.  The
     * reaction force pushes the jaws *apart*, so the gap must grow and the force
     * must fall towards zero.  With the sign inverted the loop is positive
     * feedback - a bigger force closes the jaw further, which raises the force -
     * and the jaw runs away to its mechanical stop instead.  That is not a
     * hypothetical: scenario D showed the jaws closing from a 9.2 mm gap to the
     * 1.0 mm hard stop with zero duty commanded, the force climbing from the
     * 5.013 N trip point to the 16.4 N ADC rail, which then latched a sensor
     * fault that had nothing to do with the sensor.
     */
    tp_plant_t p;
    float gap_at_release;
    float force_at_release;
    int i;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    tp_plant_set_stem_gap(&p, 6.0f);

    /* Close onto the stem until there is a real force to react against. */
    for (i = 0; i < 1060; ++i) {
        tp_plant_step(&p, 0.0f, 300.0f, PSTEP);
    }
    gap_at_release = p.gripper.gap_mm;
    force_at_release = p.gripper.force_n;
    TP_ASSERT_GT(force_at_release, 1.0f);

    /* Zero duty from here: the servo is no longer driving the jaw. */
    for (i = 0; i < 500; ++i) {
        tp_plant_step(&p, 0.0f, 0.0f, PSTEP);
    }

    TP_ASSERT_MSG(p.gripper.gap_mm > gap_at_release,
                  "the stem must push a free jaw open, not closed");
    TP_ASSERT_MSG(p.gripper.force_n < force_at_release,
                  "the force must fall once the jaw is free to open");
    TP_ASSERT_FINITE(p.gripper.gap_mm);
}
TP_TEST_CASE(plant_gripper, damage_flag_latches_above_the_threshold)
{
    tp_plant_t p;
    int i;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    tp_plant_set_stem_gap(&p, 16.0f);
    TP_ASSERT_EQ(p.gripper.damaged, 0);

    /* Drive the jaw into the hard stop: the force must exceed the 4 N damage
     * threshold and the flag must latch. */
    for (i = 0; i < 8000; ++i) {
        tp_plant_step(&p, 0.0f, 1000.0f, PSTEP);
    }
    TP_ASSERT_GT(p.gripper.peak_force_n, TP_PLANT_STEM_DAMAGE_N);
    TP_ASSERT_EQ(p.gripper.damaged, 1);
    TP_ASSERT_GT(p.gripper.damage_index, 0.0f);
}

TP_TEST_CASE(plant_gripper, jaw_never_leaves_its_mechanical_range)
{
    tp_plant_t p;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    {
        int i;
        for (i = 0; i < 5000; ++i) {
            tp_plant_step(&p, 0.0f, 1000.0f, PSTEP);
            TP_ASSERT(p.gripper.gap_mm >= TP_PLANT_JAW_HARD_STOP_MM - 1.0e-3f);
            TP_ASSERT(p.gripper.gap_mm <= TP_GRIPPER_MAX_MM + 1.0e-3f);
        }
        for (i = 0; i < 5000; ++i) {
            tp_plant_step(&p, 0.0f, -1000.0f, PSTEP);
            TP_ASSERT(p.gripper.gap_mm >= TP_PLANT_JAW_HARD_STOP_MM - 1.0e-3f);
            TP_ASSERT(p.gripper.gap_mm <= TP_GRIPPER_MAX_MM + 1.0e-3f);
        }
    }
}

/* =========================================================================
 * Joint dynamics
 * ========================================================================= */

TP_TEST_CASE(plant_joint, zero_duty_leaves_a_gravity_loaded_axis_free)
{
    tp_plant_t p;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    /* With the drive off, the 20 mN*m gravity load must accelerate the axis.
     * This is *correct* physics and it is why the application holds position in
     * IDLE rather than commanding zero duty —the first host simulation run
     * drifted at 148 mrad/s because it did not. */
    plant_run(500);
    tp_plant_step(&p, 0.0f, 0.0f, PSTEP);
    TP_ASSERT_LT(p.joint.omega_mrad_s, 0.0f);
    TP_ASSERT_LT(p.joint.theta_mrad, 0.0f);
}

TP_TEST_CASE(plant_joint, encoder_quantises_at_the_real_resolution)
{
    tp_plant_t p;
    int32_t a;
    int32_t b;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    a = tp_plant_read_encoder_count(&p);
    /* Move by less than one count: 1.534 mrad per count, so 1 mrad must not be
     * visible.  Quantisation is modelled deliberately so the velocity estimator
     * faces the same problem it does on hardware. */
    p.joint.theta_mrad += 1.0f;
    b = tp_plant_read_encoder_count(&p);
    TP_ASSERT_EQ(a, b);

    p.joint.theta_mrad += 5.0f;
    TP_ASSERT_NE(tp_plant_read_encoder_count(&p), b);
}

TP_TEST_CASE(plant_joint, soft_end_stops_hold_the_axis)
{
    tp_plant_t p;
    int i;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    for (i = 0; i < 20000; ++i) {
        tp_plant_step(&p, 1000.0f, 0.0f, PSTEP);
    }
    TP_ASSERT_NEAR(p.joint.theta_mrad, TP_JOINT_MAX_MRAD, 1.0f);
    TP_ASSERT_NEAR(p.joint.omega_mrad_s, 0.0f, 1.0f);
}

TP_TEST_CASE(plant_joint, duty_slew_limit_is_respected)
{
    tp_plant_t p;
    float previous = 0.0f;
    int i;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    /* A full-scale step must not appear in one sample: the limiter exists to
     * prevent a current spike.  At 200 000 per-mille/s, full scale is reached
     * in 5 ms, i.e. 200 per-mille per 1 ms step. */
    for (i = 0; i < 100; ++i) {
        tp_plant_step(&p, 1000.0f, 0.0f, PSTEP);
        TP_ASSERT(p.joint.pwm_permille - previous
                  <= (TP_MAX_DUTY_RATE_PER_S * PSTEP) + 1.0e-3f);
        previous = p.joint.pwm_permille;
    }
    /* And the limiter must actually be fast enough for the loop it sits in:
     * full scale in well under one period of the 8 Hz velocity loop. */
    TP_ASSERT_LT(TP_MAX_PWM_PERMILLE / TP_MAX_DUTY_RATE_PER_S, 0.02f);
}

TP_TEST_CASE(plant_joint, jam_holds_the_axis_when_the_drive_cannot_beat_it)
{
    tp_plant_t p;
    int i;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    /* The jam must exceed what the *plant's* peak-current clamp can produce:
     * 5 A x 2 N*m/A = 10 N*m.  An 8 N*m jam looks impressive but the plant can
     * beat it, so the axis moves and the test proves nothing. */
    tp_plant_set_jam(&p, 20.0f);
    TP_ASSERT_EQ(p.joint_jammed, 1);

    for (i = 0; i < 2000; ++i) {
        tp_plant_step(&p, 1000.0f, 0.0f, PSTEP);
        /* A real jam is a motion constraint: the axis must stay put, not drift.
         * Modelling it as a large torque let the joint chatter through the hold
         * band and defeated the stall detector. */
        TP_ASSERT_NEAR(p.joint.omega_mrad_s, 0.0f, 1.0e-3f);
    }
    TP_ASSERT_NEAR(p.joint.theta_mrad, TP_JOINT_HOME_MRAD, 1.0e-6f);
}

TP_TEST_CASE(plant_joint, a_weak_jam_still_allows_laboured_motion)
{
    tp_plant_t p;
    int i;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    tp_plant_set_jam(&p, 1.0f); /* well below the drive's capability */
    for (i = 0; i < 2000; ++i) {
        tp_plant_step(&p, 1000.0f, 0.0f, PSTEP);
    }
    /* The axis must move, slowly: a jam the drive can overcome is not a stall,
     * and the detector must not treat it as one. */
    TP_ASSERT_GT(p.joint.theta_mrad, 100.0f);
}

TP_TEST_CASE(plant_joint, estop_kills_the_drive_and_opens_the_jaw)
{
    tp_plant_t p;
    int i;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    /* Close the jaw first. */
    for (i = 0; i < 300; ++i) {
        tp_plant_step(&p, 0.0f, 500.0f, PSTEP);
    }
    TP_ASSERT_LT(p.gripper.gap_mm, TP_GRIPPER_MAX_MM);

    p.estop_asserted = 1;
    for (i = 0; i < 3000; ++i) {
        tp_plant_step(&p, 1000.0f, 1000.0f, PSTEP); /* commanded anyway */
    }
    /* Hardware kill: no torque regardless of the commanded duty, and the jaws
     * spring open because the design is fail-safe. */
    TP_ASSERT_NEAR(p.joint.current_a, 0.0f, 1.0e-6f);
    TP_ASSERT_NEAR(p.gripper.gap_mm, TP_GRIPPER_MAX_MM, 1.0e-2f);
}

TP_TEST_CASE(plant_sensor, force_counts_include_quantisation_and_noise)
{
    tp_plant_t p;
    uint16_t at_rest;
    uint16_t loaded;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    plant_run(50);
    at_rest = tp_plant_read_force_counts(&p);
    /* At rest the reading must sit at mid-scale, within a count. */
    TP_ASSERT_NEAR((float)at_rest, TP_FORCE_CAL_ZERO_COUNTS, 1.0f);

    p.gripper.force_n = 2.2f;
    loaded = tp_plant_read_force_counts(&p);
    /* 2.2 N at 8.02 mN per count is 274 counts. */
    TP_ASSERT_NEAR((float)loaded - (float)at_rest, 274.0f, 2.0f);
}

TP_TEST_CASE(plant_sensor, sensor_errors_are_injected_reproducibly)
{
    tp_plant_t p1;
    tp_plant_t p2;
    uint16_t a;
    uint16_t b;

    TP_ASSERT_EQ(tp_plant_init(&p1), TP_OK);
    TP_ASSERT_EQ(tp_plant_init(&p2), TP_OK);
    tp_plant_set_seed(&p1, 4242u);
    tp_plant_set_seed(&p2, 4242u);
    tp_plant_set_force_noise(&p1, 0.08f);
    tp_plant_set_force_noise(&p2, 0.08f);

    /* The internal LCG must make a seeded run reproducible: the plots and the
     * experiment numbers depend on it. */
    a = tp_plant_read_force_counts(&p1);
    b = tp_plant_read_force_counts(&p2);
    TP_ASSERT_EQ(a, b);

    tp_plant_set_sensor_error(&p1, 1.05f, 20.0f);
    p1.gripper.force_n = 1.0f;
    TP_ASSERT_NE(tp_plant_read_force_counts(&p1), (uint16_t)0u);
}

TP_TEST_CASE(plant_sensor, adc_readings_saturate_rather_than_wrap)
{
    tp_plant_t p;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    p.gripper.force_n = 1000.0f; /* absurd */
    TP_ASSERT_EQ(tp_plant_read_force_counts(&p), (uint16_t)TP_ADC_FULL_SCALE);
    p.gripper.force_n = -1000.0f;
    TP_ASSERT_EQ(tp_plant_read_force_counts(&p), (uint16_t)0u);
}

TP_TEST_CASE(plant_sensor, io_bitmask_polarity_is_fail_safe)
{
    tp_plant_t p;
    uint8_t mask;

    TP_ASSERT_EQ(tp_plant_init(&p), TP_OK);
    mask = tp_plant_read_io(&p);
    /* E-stop is a normally-closed loop: it reads HIGH while the loop is intact
     * and DROPS when the button is pressed or the wire breaks. */
    TP_ASSERT((mask & (1u << (int)TP_IO_ESTOP)) != 0u);

    p.estop_asserted = 1;
    mask = tp_plant_read_io(&p);
    TP_ASSERT((mask & (1u << (int)TP_IO_ESTOP)) == 0u);
}

/* =========================================================================
 * Device drivers against the plant
 * ========================================================================= */

TP_TEST_CASE(driver_motor, slew_limit_is_respected_and_the_command_arrives)
{
    tp_motor_t m;
    float previous = 0.0f;
    int i;

    TP_ASSERT_EQ(tp_platform_init(), TP_OK);
    TP_ASSERT_EQ(tp_motor_init(&m, TP_MOTOR_JOINT, TP_CONTROL_DT_S), TP_OK);
    /* Raise the current limit so this test isolates the *slew limiter*.  At the
     * production 3 A limit a 300 per-mille command draws 3.6 A into a stationary
     * winding and the fold-back correctly settles it at ~225 per-mille instead,
     * which is the subject of the next test.  Conflating the two made this test
     * look like a slew-limiter failure when it was a working current limiter. */
    m.current_limit_ma = 5000.0f;

    TP_ASSERT_EQ(tp_motor_set_duty(&m, 300.0f), TP_OK);
    for (i = 0; i < 100; ++i) {
        (void)tp_motor_update(&m, TP_CONTROL_DT_S);
        TP_ASSERT(tp_motor_get_duty(&m) - previous
                  <= (TP_MAX_DUTY_RATE_PER_S * TP_CONTROL_DT_S) + 1.0e-3f);
        previous = tp_motor_get_duty(&m);
        tp_host_platform_advance(1000u);
    }
    /* The limiter must have let the command through in a few milliseconds. */
    TP_ASSERT_NEAR(tp_motor_get_duty(&m), 300.0f, 1.0f);
    TP_ASSERT_EQ(tp_motor_is_faulted(&m), 0);

    /* A non-finite command must be refused, not written to the timer register. */
    TP_ASSERT_EQ(tp_motor_set_duty(&m, 0.0f / 0.0f), TP_ERR_RANGE);
}

TP_TEST_CASE(driver_motor, foldback_regulates_instead_of_tripping)
{
    tp_motor_t m;
    tp_plant_t *plant = tp_host_platform_plant();
    int i;
    float peak_current = 0.0f;

    TP_ASSERT_EQ(tp_platform_init(), TP_OK);
    TP_ASSERT_EQ(tp_motor_init(&m, TP_MOTOR_JOINT, TP_CONTROL_DT_S), TP_OK);

    /* Hold the axis so a full-scale command produces stall current, then command
     * full scale anyway.  The fold-back must bring the current down to about
     * 90 % of the limit and *keep* it there: a current limit that trips instead
     * of regulating turns every heavy move into a latched fault, which is not a
     * protection, it is an outage. */
    tp_plant_set_jam(plant, 50.0f);
    TP_ASSERT_EQ(tp_motor_set_duty(&m, 1000.0f), TP_OK);
    for (i = 0; i < 400; ++i) {
        (void)tp_motor_update(&m, TP_CONTROL_DT_S);
        tp_host_platform_advance(1000u);
        if (TP_FABS(tp_motor_get_current_ma(&m)) > peak_current) {
            peak_current = TP_FABS(tp_motor_get_current_ma(&m));
        }
    }
    TP_ASSERT_EQ(tp_motor_is_faulted(&m), 0);
    TP_ASSERT_GT(m.foldback_events, 0u);
    /* Settled below the limit, and the duty was genuinely reduced. */
    TP_ASSERT_LT(TP_FABS(tp_motor_get_current_ma(&m)), TP_MAX_CURRENT_MA);
    TP_ASSERT_LT(tp_motor_get_duty(&m), 1000.0f);
}

TP_TEST_CASE(driver_motor, overcurrent_disables_the_drive_when_foldback_cannot_help)
{
    tp_motor_t m;
    tp_plant_t *plant = tp_host_platform_plant();
    int i;
    int tripped = 0;

    TP_ASSERT_EQ(tp_platform_init(), TP_OK);
    TP_ASSERT_EQ(tp_motor_init(&m, TP_MOTOR_JOINT, TP_CONTROL_DT_S), TP_OK);

    /*
     * The trip is a *last resort*: it fires only when the fold-back cannot bring
     * the current under the limit.  That happens when the drive is configured
     * for a current the load cannot respect even at minimum duty, so the test
     * configures exactly that — a 100 mA limit against a winding that draws
     * 600 mA at the 5 % duty floor.  Commanding full scale into the production
     * 3 A limit does *not* trip, because the fold-back succeeds; asserting that
     * it does was the first version of this test and it was asserting the wrong
     * behaviour.
     */
    m.current_limit_ma = 100.0f;
    tp_plant_set_jam(plant, 50.0f);
    TP_ASSERT_EQ(tp_motor_set_duty(&m, 1000.0f), TP_OK);

    for (i = 0; i < 500; ++i) {
        (void)tp_motor_update(&m, TP_CONTROL_DT_S);
        tp_host_platform_advance(1000u);
        if (tp_motor_is_faulted(&m) != 0) {
            tripped = 1;
            break;
        }
    }
    TP_ASSERT_MSG(tripped != 0, "over-current protection never tripped");
    /* A faulted channel must also be de-energised and must refuse further
     * commands: a protection that only sets a flag is not a protection. */
    TP_ASSERT_EQ(tp_motor_is_enabled(&m), 0);
    TP_ASSERT_EQ(tp_motor_set_duty(&m, 100.0f), TP_ERR_FAULT_ACTIVE);
    TP_ASSERT_NEAR(tp_motor_get_duty(&m), 0.0f, 1.0e-6f);
}
TP_TEST_CASE(driver_motor, clear_is_refused_while_the_condition_persists)
{
    tp_motor_t m;
    tp_plant_t *plant = tp_host_platform_plant();
    int i;
    int tripped = 0;

    TP_ASSERT_EQ(tp_platform_init(), TP_OK);
    TP_ASSERT_EQ(tp_motor_init(&m, TP_MOTOR_JOINT, TP_CONTROL_DT_S), TP_OK);
    /* Same configuration as the trip test: a limit the fold-back cannot meet. */
    m.current_limit_ma = 100.0f;
    tp_plant_set_jam(plant, 50.0f);
    (void)tp_motor_set_duty(&m, 1000.0f);
    for (i = 0; i < 500; ++i) {
        (void)tp_motor_update(&m, TP_CONTROL_DT_S);
        tp_host_platform_advance(1000u);
        if (tp_motor_is_faulted(&m) != 0) {
            tripped = 1;
            break;
        }
    }
    TP_ASSERT_EQ(tripped, 1);

    /* The current is still above the limit, so an acknowledgement must not be
     * able to defeat the protection. */
    TP_ASSERT_EQ(tp_motor_clear_fault(&m), TP_ERR_FAULT_ACTIVE);
    TP_ASSERT_EQ(tp_motor_is_faulted(&m), 1);

    /* Release the jam and let the current fall, then the clear must succeed. */
    tp_plant_set_jam(plant, 0.0f);
    for (i = 0; i < 200; ++i) {
        /* The update call matters: it is what refreshes `current_ma` from the
         * ADC.  Advancing the plant alone leaves the measured value stale at its
         * last reading, and `clear_fault` would then refuse forever on a current
         * that is no longer flowing — which is exactly how this test first
         * failed, and what the "keep sampling while disabled" fix addresses. */
        (void)tp_motor_update(&m, TP_CONTROL_DT_S);
        tp_host_platform_advance(1000u);
    }
    TP_ASSERT_EQ(tp_motor_clear_fault(&m), TP_OK);
    TP_ASSERT_EQ(tp_motor_is_faulted(&m), 0);
    TP_ASSERT_EQ(tp_motor_enable(&m), TP_OK);
}
TP_TEST_CASE(driver_motor, disable_then_enable_requires_a_clear)
{
    tp_motor_t m;

    TP_ASSERT_EQ(tp_platform_init(), TP_OK);
    TP_ASSERT_EQ(tp_motor_init(&m, TP_MOTOR_GRIPPER, TP_CONTROL_DT_S), TP_OK);
    TP_ASSERT_EQ(tp_motor_disable(&m), TP_OK);
    TP_ASSERT_EQ(tp_motor_is_enabled(&m), 0);
    TP_ASSERT_EQ(tp_motor_enable(&m), TP_OK);
    TP_ASSERT_EQ(tp_motor_is_enabled(&m), 1);
    TP_ASSERT_EQ(tp_motor_init(NULL, TP_MOTOR_JOINT, TP_CONTROL_DT_S),
                 TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_motor_init(&m, (tp_motor_ch_t)99, TP_CONTROL_DT_S),
                 TP_ERR_PARAM);
}

TP_TEST_CASE(driver_encoder, velocity_estimator_handles_low_speed)
{
    tp_encoder_t enc;
    int i;
    float v;

    TP_ASSERT_EQ(tp_platform_init(), TP_OK);
    TP_ASSERT_EQ(tp_encoder_init(&enc, TP_ENCODER_JOINT), TP_OK);
    TP_ASSERT_EQ(tp_encoder_set_zero(&enc), TP_OK);

    /* Drive the plant slowly and confirm the estimator reports a *non-zero*
     * velocity.  A naive delta/dt estimator reads exactly zero below about
     * 770 mrad/s at 500 Hz, which is where the stall detector has to work. */
    {
        tp_plant_t *p = tp_host_platform_plant();
        for (i = 0; i < 200; ++i) {
            tp_plant_step(p, 40.0f, 0.0f, PSTEP);
            tp_host_platform_advance(1000u);
            (void)tp_encoder_update(&enc, tp_platform_time_ms(), 2000u);
        }
    }
    v = tp_encoder_get_speed_mrad_s(&enc);
    TP_ASSERT_GT(v, 1.0f);
    TP_ASSERT_FINITE(v);
    TP_ASSERT_EQ(tp_encoder_is_faulty(&enc), 0);
}

TP_TEST_CASE(driver_encoder, zeroing_clears_position_and_velocity)
{
    tp_encoder_t enc;

    TP_ASSERT_EQ(tp_platform_init(), TP_OK);
    TP_ASSERT_EQ(tp_encoder_init(&enc, TP_ENCODER_JOINT), TP_OK);
    TP_ASSERT_EQ(tp_encoder_set_zero(&enc), TP_OK);
    TP_ASSERT_NEAR(tp_encoder_get_position_mrad(&enc), 0.0f, 1.0e-6f);
    TP_ASSERT_NEAR(tp_encoder_get_speed_mrad_s(&enc), 0.0f, 1.0e-6f);
    TP_ASSERT_EQ(tp_encoder_mrad_to_counts(&enc, TP_MRAD_PER_COUNT), 1);
    TP_ASSERT_EQ(tp_encoder_get_position_mrad(NULL), 0.0f);
}

TP_TEST_CASE(driver_gripper, limit_switch_vetoes_further_motion)
{
    tp_gripper_t g;
    int i;

    TP_ASSERT_EQ(tp_platform_init(), TP_OK);
    TP_ASSERT_EQ(tp_gripper_init(&g, TP_CONTROL_DT_S), TP_OK);

    /* Homing leaves the jaw sitting on the open switch.  Commanding it further
     * open must be *vetoed* rather than stalling the motor against the stop:
     * the veto lives in the driver so that no caller can forget it.
     *
     * Note that commanding the jaws *open* through tp_gripper_open() would not
     * exercise this: the jaw is already at the target, so the position loop
     * commands zero velocity and there is nothing to veto.  The test has to
     * issue an actual velocity command. */
    TP_ASSERT_EQ(tp_gripper_set_velocity_mm_s(&g, TP_GRIPPER_MAX_SPEED_MM_S),
                 TP_OK);
    for (i = 0; i < 50; ++i) {
        (void)tp_gripper_update(&g, TP_CONTROL_DT_S);
        tp_host_platform_advance(1000u);
    }
    TP_ASSERT_GT(g.limit_events, 0u);
    TP_ASSERT_EQ(g.veto_active, 1);
    /* The *request* must survive the veto; only the applied value is zeroed.
     * Latching the veto into the setpoint is the bug this asserts against. */
    TP_ASSERT_NEAR(g.cmd_velocity_mm_s, TP_GRIPPER_MAX_SPEED_MM_S, 1.0e-3f);
    TP_ASSERT_NEAR(g.applied_velocity_mm_s, 0.0f, 1.0e-3f);
    TP_ASSERT_NEAR(g.cmd_duty, 0.0f, 1.0e-3f);
}
TP_TEST_CASE(driver_gripper, close_then_open_returns_to_open)
{
    tp_gripper_t g;
    int i;

    TP_ASSERT_EQ(tp_platform_init(), TP_OK);
    TP_ASSERT_EQ(tp_gripper_init(&g, TP_CONTROL_DT_S), TP_OK);
    TP_ASSERT_EQ(tp_gripper_close(&g), TP_OK);
    for (i = 0; i < 3000; ++i) {
        (void)tp_gripper_update(&g, TP_CONTROL_DT_S);
        tp_host_platform_advance(1000u);
    }
    TP_ASSERT_LT(tp_gripper_get_gap_mm(&g), TP_GRIPPER_MAX_MM - 5.0f);

    TP_ASSERT_EQ(tp_gripper_open(&g), TP_OK);
    for (i = 0; i < 6000; ++i) {
        (void)tp_gripper_update(&g, TP_CONTROL_DT_S);
        tp_host_platform_advance(1000u);
    }
    TP_ASSERT_EQ(tp_gripper_is_open(&g), 1);
}

TP_TEST_CASE(driver_gripper, invalid_velocity_is_rejected)
{
    tp_gripper_t g;

    TP_ASSERT_EQ(tp_platform_init(), TP_OK);
    TP_ASSERT_EQ(tp_gripper_init(&g, TP_CONTROL_DT_S), TP_OK);
    TP_ASSERT_EQ(tp_gripper_set_velocity_mm_s(&g, 1.0e6f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_gripper_move_to_mm(&g, 10.0f, -1.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_gripper_move_to_mm(&g, 10.0f, 0.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_gripper_set_velocity_mm_s(NULL, 1.0f), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_gripper_get_gap_mm(NULL), 0.0f);
}

TP_TEST_CASE(driver_limit_switch, debounces_a_bouncing_contact)
{
    tp_limit_switch_t sw;

    TP_ASSERT_EQ(tp_platform_init(), TP_OK);
    TP_ASSERT_EQ(tp_limit_switch_init(&sw, TP_IO_LIMIT_JOINT_MIN, 0), TP_OK);
    TP_ASSERT_EQ(tp_limit_switch_is_active(&sw), 0);

    /* Simulate a bouncy contact by forcing the logical level; the debouncer
     * must not publish a state change until the samples agree. */
    tp_limit_switch_force(&sw, 1);
    TP_ASSERT_EQ(tp_limit_switch_is_active(&sw), 1);
    tp_limit_switch_force(&sw, 0);
    TP_ASSERT_EQ(tp_limit_switch_is_active(&sw), 0);

    TP_ASSERT_EQ(tp_limit_switch_init(&sw, (tp_io_id_t)99, 0), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_limit_switch_init(NULL, TP_IO_ESTOP, 1), TP_ERR_PARAM);
}
