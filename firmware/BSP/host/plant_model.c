/**
 * @file    plant_model.c
 * @brief   Implementation of the host-side physics model.
 * @copyright MIT License, see LICENSE.
 */

#include "plant_model.h"

#include <string.h>

#include "bsp_types.h"
#include "compiler.h"
#include "config.h"
#include "logger.h"

/** @brief Two pi, spelled out to avoid a libm dependency in the plant. */
#define PLANT_TWO_PI 6.28318530717959f

/** @brief Milliardians per radian. */
#define PLANT_MRAD_PER_RAD 1000.0f

/**
 * @brief Advance the deterministic pseudo-random generator (LCG).
 *
 * Numerical Recipes' constants: cheap, adequate for noise injection and,
 * crucially, reproducible across machines — unlike `rand()`.
 *
 * @param[in,out] state  Generator state, must not be 0.
 * @return Next 32-bit value.
 */
static uint32_t plant_rand_u32(uint32_t *state)
{
    *state = (*state * 1664525u) + 1013904223u;
    return *state;
}

/**
 * @brief Uniform sample in [0, 1).
 *
 * @param[in,out] state  Generator state.
 * @return Value in [0, 1).
 */
static float plant_rand_unit(uint32_t *state)
{
    return (float)(plant_rand_u32(state) >> 8) * (1.0f / 16777216.0f);
}

/**
 * @brief Approximate standard-normal sample via the sum of 6 uniforms.
 *
 * The central limit theorem gives a mean of 3 and a variance of 0.5 for six
 * uniforms, so `(sum - 3) / sqrt(0.5)` is normalised.  Using 6 terms instead
 * of the usual 12 keeps it cheap while staying close enough to Gaussian for
 * noise-injection purposes; the truncation at +-2.45 sigma is irrelevant here.
 *
 * @param[in,out] state  Generator state.
 * @return Zero-mean unit-variance sample.
 */
static float plant_rand_normal(uint32_t *state)
{
    float sum = 0.0f;
    int i;
    for (i = 0; i < 6; ++i) {
        sum += plant_rand_unit(state);
    }
    return (sum - 3.0f) * 1.4142136f;
}

/**
 * @brief Clamp a float into a range.
 *
 * @param[in] v   Value.
 * @param[in] lo  Lower bound.
 * @param[in] hi  Upper bound.
 * @return Clamped value.
 */
static float plant_clampf(float v, float lo, float hi)
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
 * @brief Clamp a float to the 12-bit ADC conversion range.
 *
 * @param[in] v  Count value.
 * @return Value in [0, 4095].
 */
static uint16_t plant_adc_saturate(float v)
{
    if (!TP_ISFINITE(v) || (v <= 0.0f)) {
        return 0u;
    }
    if (v >= TP_ADC_FULL_SCALE) {
        return (uint16_t)TP_ADC_FULL_SCALE;
    }
    return (uint16_t)(v + 0.5f);
}

tp_status_t tp_plant_init(tp_plant_t *p)
{
    if (p == NULL) {
        return TP_ERR_PARAM;
    }
    (void)memset(p, 0, sizeof(*p));

    p->joint.theta_mrad = TP_JOINT_HOME_MRAD;
    p->joint.omega_mrad_s = 0.0f;
    p->joint.current_a = 0.0f;
    p->joint.pwm_permille = 0.0f;
    /* A small constant load stands in for gravity on the arm.  It is what
     * makes the integral term of the position loop observable: with a pure P
     * controller and a perfect inner loop there would be no steady-state
     * error to reject, and Experiment 01 would prove nothing. */
    p->joint.load_torque_nm = 0.02f;

    p->gripper.gap_mm = TP_GRIPPER_MAX_MM;
    p->gripper.gap_vel_mm_s = 0.0f;
    p->gripper.force_n = 0.0f;
    p->gripper.peak_force_n = 0.0f;
    p->gripper.damage_index = 0.0f;
    p->gripper.damaged = 0;
    p->gripper.stem_gap_mm = TP_PLANT_STEM_GAP_MM;

    p->vbus_v = 24.0f;
    p->tick_ms = 0u;
    p->joint_jammed = 0;
    p->jam_torque_nm = 0.0f;
    p->encoder_fault = 0;
    p->force_noise_sigma_n = 0.0f;
    p->adc_gain_error = 1.0f;
    p->adc_zero_offset_counts = 0.0f;
    p->estop_asserted = 0;
    p->noise_seed = 12345u;
    return TP_OK;
}

void tp_plant_reset(tp_plant_t *p)
{
    uint32_t seed;

    if (p == NULL) {
        return;
    }
    seed = (p->noise_seed != 0u) ? p->noise_seed : 12345u;
    (void)tp_plant_init(p);
    p->noise_seed = seed;
}

void tp_plant_step(tp_plant_t *p, float pwm_joint, float pwm_gripper, float dt_s)
{
    float omega_rad_s;
    float pwm_target;
    float max_step;
    float v_applied;
    float back_emf_v;
    float i_cmd_a;
    float tau_e;
    float alpha_e;
    float torque_nm;
    float friction_nm;
    float resisting_nm;
    float alpha_jaw;
    float v_jaw;
    float penetration_mm;
    float force_n;
    int   jam_holds;

    if ((p == NULL) || !TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return;
    }

    /* =====================================================================
     * Joint axis
     * ===================================================================== */

    /* PWM slew limiting.  A real gate driver plus a 1.5 mH winding cannot step
     * the applied voltage instantaneously, and the D-term of a velocity loop
     * would otherwise be able to command a full-scale step between two
     * consecutive samples. */
    pwm_target = plant_clampf(pwm_joint, -TP_MAX_PWM_PERMILLE, TP_MAX_PWM_PERMILLE);
    max_step = TP_MAX_DUTY_RATE_PER_S * dt_s;
    if ((pwm_target - p->joint.pwm_permille) > max_step) {
        p->joint.pwm_permille += max_step;
    } else if ((p->joint.pwm_permille - pwm_target) > max_step) {
        p->joint.pwm_permille -= max_step;
    } else {
        p->joint.pwm_permille = pwm_target;
    }

    if (p->estop_asserted != 0) {
        /* Hardware kill: the driver's enable pin is low, so the winding is
         * open and no torque is produced regardless of the commanded duty. */
        p->joint.pwm_permille = 0.0f;
        p->joint.current_a = 0.0f;
    } else {
        v_applied = (p->joint.pwm_permille / TP_MAX_PWM_PERMILLE) * p->vbus_v;
        omega_rad_s = p->joint.omega_mrad_s / PLANT_MRAD_PER_RAD;
        back_emf_v = TP_PLANT_MOTOR_KE_VS_RAD * omega_rad_s;

        /* Steady-state current from Ohm's law; negative values mean the motor
         * is regenerating, which is physically possible but blocked by the
         * driver's body diodes in this simplified model, so it is clamped. */
        i_cmd_a = (v_applied - back_emf_v) / TP_PLANT_MOTOR_R_OHM;
        i_cmd_a = plant_clampf(i_cmd_a, -TP_PEAK_CURRENT_MA / 1000.0f,
                               TP_PEAK_CURRENT_MA / 1000.0f);

        tau_e = TP_PLANT_MOTOR_TAU_E_S;
        alpha_e = dt_s / (tau_e + dt_s);
        p->joint.current_a += (i_cmd_a - p->joint.current_a) * alpha_e;

        /* Continuous current limit: the driver folds back rather than
         * tripping, which is what the over-current detector then notices. */
        p->joint.current_a = plant_clampf(p->joint.current_a,
                                          -TP_PEAK_CURRENT_MA / 1000.0f,
                                          TP_PEAK_CURRENT_MA / 1000.0f);
    }

    torque_nm = TP_PLANT_MOTOR_KT_NM_PER_A * p->joint.current_a;

    /* Coulomb + viscous friction.  The tanh removes the discontinuity at zero
     * speed that would otherwise chatter the integrator at 1 kHz. */
    omega_rad_s = p->joint.omega_mrad_s / PLANT_MRAD_PER_RAD;
    {
        float x = omega_rad_s / TP_PLANT_JOINT_OMEGA_EPS_RAD_S;
        float x2 = x * x;
        float tanh_x = x * (27.0f + x2) / (27.0f + 9.0f * x2); /* Pade approx. */
        friction_nm = (TP_PLANT_JOINT_TC_NM * tanh_x)
                    + (TP_PLANT_JOINT_B_NMS_RAD * omega_rad_s);
    }

    resisting_nm = p->joint.load_torque_nm;
    jam_holds = 0;
    if (p->joint_jammed != 0) {
        /*
         * Stick-slip model, not a large torque.
         *
         * The first two attempts at this both modelled the jam as a torque that
         * opposes the *velocity*, and both failed for a numerical reason that is
         * worth recording: an 8 N*m resisting torque on a 2.5e-3 kg*m^2 inertia
         * is an acceleration of 3200 rad/s^2, so a single 1 ms explicit-Euler
         * step changes the velocity by 3.2 rad/s.  The "held" condition required
         * |omega| <= 1e-4 rad/s - a band four orders of magnitude narrower than
         * what one step can resolve - so the state chattered through it every
         * sample and the axis oscillated with an amplitude of thousands of
         * milliradians per second instead of staying stuck.
         *
         * A jam is not a torque, it is a *motion constraint*: the axis is stuck
         * while the drive cannot overcome the jam's static friction, and it
         * moves (resisted by that same friction) once it can.  Testing the
         * condition on the applied torque rather than on the velocity removes
         * the chatter entirely, because the torque is a smooth, bounded quantity
         * that does not oscillate.
         *
         * Note that a jam *torque below the drive's capability* produces a
         * slow, laboured motion rather than a stall - which is correct, and is
         * why scenario F uses 8 N*m against a 6 N*m drive.
         */
        if (TP_FABS(torque_nm) <= p->jam_torque_nm) {
            jam_holds = 1;
            torque_nm = 0.0f;
            friction_nm = 0.0f;
            resisting_nm = 0.0f;
        } else {
            resisting_nm += (torque_nm > 0.0f) ? p->jam_torque_nm
                                               : -p->jam_torque_nm;
        }
    }

    if (jam_holds != 0) {
        p->joint.omega_mrad_s = 0.0f;
    } else {
        float net_nm = torque_nm - friction_nm - resisting_nm;
        float alpha_rad_s2 = net_nm / TP_PLANT_JOINT_J_KGM2;
        float omega_rad_s_next = omega_rad_s + (alpha_rad_s2 * dt_s);
        p->joint.omega_mrad_s = omega_rad_s_next * PLANT_MRAD_PER_RAD;
        p->joint.theta_mrad += p->joint.omega_mrad_s * dt_s;
    }

    /* Soft end-stops: the real machine has mechanical limits, and hitting one
     * must not let the model integrate to a nonsense angle. */
    if (p->joint.theta_mrad < TP_JOINT_MIN_MRAD) {
        p->joint.theta_mrad = TP_JOINT_MIN_MRAD;
        if (p->joint.omega_mrad_s < 0.0f) {
            p->joint.omega_mrad_s = 0.0f;
        }
    } else if (p->joint.theta_mrad > TP_JOINT_MAX_MRAD) {
        p->joint.theta_mrad = TP_JOINT_MAX_MRAD;
        if (p->joint.omega_mrad_s > 0.0f) {
            p->joint.omega_mrad_s = 0.0f;
        }
    }

    /* =====================================================================
     * Gripper axis and tea stem
     * ===================================================================== */

    alpha_jaw = dt_s / (TP_PLANT_JAW_TAU_S + dt_s);
    v_jaw = p->gripper.gap_vel_mm_s;

    if (p->estop_asserted != 0) {
        /* Jaws are sprung open on loss of power: a fail-safe design choice
         * documented in hardware/interface_design.md. */
        v_jaw = TP_GRIPPER_MAX_SPEED_MM_S;
    } else {
        /* Sign convention: a *positive* duty closes the jaws, i.e. reduces the
         * gap.  Inverting here is what makes this model agree with the driver's
         * documented convention in `gripper.h`; getting it wrong makes the
         * gripper open when commanded to close, and the resulting "no contact
         * within timeout" fault looks like a controller problem when it is
         * actually a convention mismatch.  A unit test asserts that
         * `tp_plant_gripper_velocity_to_pwm` and the driver's internal mapping
         * agree, so the two cannot drift apart again. */
        float v_cmd = -(pwm_gripper / TP_MAX_PWM_PERMILLE)
                    * TP_GRIPPER_MAX_SPEED_MM_S;
        v_jaw += (v_cmd - v_jaw) * alpha_jaw;
        /* Force back-drive: the servo is stiff but not infinitely so. */
        v_jaw -= p->gripper.force_n / TP_PLANT_JAW_BACKDRIVE;
    }

    p->gripper.gap_vel_mm_s = v_jaw;
    p->gripper.gap_mm += v_jaw * dt_s;

    if (p->gripper.gap_mm < TP_PLANT_JAW_HARD_STOP_MM) {
        p->gripper.gap_mm = TP_PLANT_JAW_HARD_STOP_MM;
        if (p->gripper.gap_vel_mm_s < 0.0f) {
            p->gripper.gap_vel_mm_s = 0.0f;
        }
    } else if (p->gripper.gap_mm > TP_GRIPPER_MAX_MM) {
        p->gripper.gap_mm = TP_GRIPPER_MAX_MM;
        if (p->gripper.gap_vel_mm_s > 0.0f) {
            p->gripper.gap_vel_mm_s = 0.0f;
        }
    }

    penetration_mm = p->gripper.stem_gap_mm - p->gripper.gap_mm;
    if (penetration_mm > 0.0f) {
        float damping = TP_PLANT_STEM_DAMP_N_S_MM
                      * ((p->gripper.gap_vel_mm_s < 0.0f)
                             ? -p->gripper.gap_vel_mm_s : 0.0f);
        force_n = (TP_PLANT_STEM_K1_N_PER_MM * penetration_mm)
                + (TP_PLANT_STEM_K2_N_PER_MM2 * penetration_mm * penetration_mm)
                + damping;
    } else {
        force_n = 0.0f;
    }
    if (!TP_ISFINITE(force_n) || (force_n < 0.0f)) {
        force_n = 0.0f;
    }
    p->gripper.force_n = force_n;

    if (force_n > p->gripper.peak_force_n) {
        p->gripper.peak_force_n = force_n;
    }
    if (force_n > TP_PLANT_STEM_DAMAGE_N) {
        /* Accumulated damage: a short excursion is less harmful than a long
         * one, so an integral is more informative than a bare flag. */
        p->gripper.damage_index += (force_n - TP_PLANT_STEM_DAMAGE_N) * dt_s;
        p->gripper.damaged = 1;
    }

    p->tick_ms += (uint32_t)(dt_s * 1000.0f + 0.5f);
}

float tp_plant_gripper_velocity_to_pwm(float v_mm_s)
{
    float duty;

    if (!TP_ISFINITE(v_mm_s)) {
        return 0.0f;
    }
    /* Positive duty closes (i.e. reduces the gap), and a negative velocity means
     * closing, hence the negation.  This is the exact inverse of the plant's
     * own duty -> velocity mapping above. */
    duty = -(v_mm_s / TP_GRIPPER_MAX_SPEED_MM_S) * TP_MAX_PWM_PERMILLE;
    return plant_clampf(duty, -TP_MAX_PWM_PERMILLE, TP_MAX_PWM_PERMILLE);
}

int32_t tp_plant_read_encoder_count(const tp_plant_t *p)
{
    float counts;
    float zero;

    if (p == NULL) {
        return 0;
    }
    zero = TP_JOINT_HOME_MRAD;
    counts = ((p->joint.theta_mrad - zero) / TP_MRAD_PER_COUNT);

    if (p->encoder_fault != 0) {
        /* Report a value that is far enough from the truth to be detected as
         * implausible by the accelerometer-style check in safety_monitor.c. */
        counts = counts + 30000.0f + (float)(p->tick_ms % 500u) * 7.0f;
    }
    if (counts > 2147483647.0f) {
        return 2147483647;
    }
    if (counts < -2147483648.0f) {
        return -2147483648;
    }
    /* Truncation towards zero is what a hardware quadrature counter does. */
    return (int32_t)counts;
}

uint16_t tp_plant_read_force_counts(tp_plant_t *p)
{
    float counts;
    float n_per_count;

    if (p == NULL) {
        return 0u;
    }
    n_per_count = TP_FORCE_CAL_SCALE_N_PER_COUNT;
    if (!(n_per_count > 0.0f)) {
        return 0u;
    }

    counts = TP_FORCE_CAL_ZERO_COUNTS + (p->gripper.force_n / n_per_count);

    /* Sensor imperfections, applied in the analogue domain so that the
     * calibration routine has something real to correct. */
    counts = (counts - TP_FORCE_CAL_ZERO_COUNTS) * p->adc_gain_error
           + TP_FORCE_CAL_ZERO_COUNTS
           + p->adc_zero_offset_counts;

    if (p->force_noise_sigma_n > 0.0f) {
        float noise_n = plant_rand_normal(&p->noise_seed) * p->force_noise_sigma_n;
        counts += noise_n / n_per_count;
    }

    return plant_adc_saturate(counts);
}

uint16_t tp_plant_read_current_counts(const tp_plant_t *p, int ch)
{
    float current_a;
    float counts;

    if (p == NULL) {
        return 2048u;
    }
    current_a = (ch == (int)TP_MOTOR_GRIPPER)
              ? (p->gripper.gap_vel_mm_s / TP_GRIPPER_MAX_SPEED_MM_S) * 0.5f
              : p->joint.current_a;

    /* Bidirectional shunt amplifier: 2048 counts is zero, +-1000 counts
     * corresponds to +-5 A (i.e. 0.2 counts per mA, 5 mA per count). */
    counts = 2048.0f + (current_a / 5.0f) * 1000.0f;
    return plant_adc_saturate(counts);
}

uint16_t tp_plant_read_vbus_counts(const tp_plant_t *p)
{
    float v_adc;

    if (p == NULL) {
        return 0u;
    }
    /* 10:1 divider, so 24 V appears as 2.4 V at the pin. */
    v_adc = (p->vbus_v / 10.0f);
    return plant_adc_saturate((v_adc / TP_ADC_VREF_V) * (TP_ADC_FULL_SCALE + 1.0f));
}

uint8_t tp_plant_read_io(const tp_plant_t *p)
{
    uint8_t mask = 0u;

    if (p == NULL) {
        return 0u;
    }
    /* E-stop is a normally-closed loop: the input reads high (1) while the
     * loop is intact and drops to 0 when the button is pressed or the wire
     * breaks.  safety_monitor.c applies that polarity. */
    if (p->estop_asserted == 0) {
        mask |= (uint8_t)(1u << (int)TP_IO_ESTOP);
    }
    if (p->joint.theta_mrad <= (TP_JOINT_MIN_MRAD + 200.0f)) {
        mask |= (uint8_t)(1u << (int)TP_IO_LIMIT_JOINT_MIN);
    }
    if (p->joint.theta_mrad >= (TP_JOINT_MAX_MRAD - 200.0f)) {
        mask |= (uint8_t)(1u << (int)TP_IO_LIMIT_JOINT_MAX);
    }
    if (p->gripper.gap_mm >= (TP_GRIPPER_MAX_MM - 0.2f)) {
        mask |= (uint8_t)(1u << (int)TP_IO_LIMIT_GRIPPER_OPEN);
    }
    if (p->gripper.gap_mm <= (TP_PLANT_JAW_HARD_STOP_MM + 0.2f)) {
        mask |= (uint8_t)(1u << (int)TP_IO_LIMIT_GRIPPER_CLOSE);
    }
    /* nFAULT from the gate driver is active low, so it reads high when OK. */
    mask |= (uint8_t)(1u << (int)TP_IO_DRIVER_FAULT);
    return mask;
}

void tp_plant_set_jam(tp_plant_t *p, float torque_nm)
{
    if (p == NULL) {
        return;
    }
    p->joint_jammed = (torque_nm > 0.0f) ? 1 : 0;
    p->jam_torque_nm = (torque_nm > 0.0f) ? torque_nm : 0.0f;
}

void tp_plant_set_encoder_fault(tp_plant_t *p, int enable)
{
    if (p == NULL) {
        return;
    }
    p->encoder_fault = (enable != 0) ? 1 : 0;
}

void tp_plant_set_force_noise(tp_plant_t *p, float sigma_n)
{
    if (p == NULL) {
        return;
    }
    p->force_noise_sigma_n = (TP_ISFINITE(sigma_n) && (sigma_n > 0.0f))
                           ? sigma_n : 0.0f;
}

void tp_plant_set_sensor_error(tp_plant_t *p, float gain_error, float offset_counts)
{
    if (p == NULL) {
        return;
    }
    p->adc_gain_error = (TP_ISFINITE(gain_error) && (gain_error > 0.0f))
                      ? gain_error : 1.0f;
    p->adc_zero_offset_counts = TP_ISFINITE(offset_counts) ? offset_counts : 0.0f;
}

void tp_plant_set_stem_gap(tp_plant_t *p, float gap_mm)
{
    if (p == NULL) {
        return;
    }
    if (!TP_ISFINITE(gap_mm)) {
        return;
    }
    p->gripper.stem_gap_mm = plant_clampf(gap_mm, TP_PLANT_JAW_HARD_STOP_MM,
                                          TP_GRIPPER_MAX_MM);
}

void tp_plant_set_seed(tp_plant_t *p, uint32_t seed)
{
    if (p == NULL) {
        return;
    }
    p->noise_seed = (seed != 0u) ? seed : 1u;
}
