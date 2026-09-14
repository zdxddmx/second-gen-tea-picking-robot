/**
 * @file    plant_model.h
 * @brief   Deterministic physics model of the robot, used by the host backend.
 *
 * @details
 * # What this is
 * A continuous-time model of the two actuators plus the tea stem, integrated
 * with explicit Euler at the firmware's own control period.  It is the thing
 * that makes `TP_PLATFORM_HOST` a *simulation* rather than a *mock*: the same
 * `motor.c`, `pid.c`, `robot_fsm.c` and `safety_monitor.c` sources that run on
 * the STM32 run here, seeing a plausible joint instead of a stub returning
 * zero.
 *
 * # Why it is not a mock
 * A mock would let the control loop "succeed" no matter how wrong the
 * controller is, which is worse than no test at all.  This model has real
 * inertia, real current limits, real Coulomb friction, a quantised encoder and
 * a nonlinear stem stiffness, so a badly tuned PID overshoots *in simulation
 * exactly as it would on the bench*.  That is what makes Experiment 01-03 in
 * `docs/11_experiment_design.md` meaningful.
 *
 * # Model (joint axis)
 * @verbatim
 *   V        = pwm_permille / 1000 * V_bus                     (slew limited)
 *   i_cmd    = (V - Ke * omega) / R                            (Ohm + back-EMF)
 *   di/dt    = (i_cmd - i) / tau_e          with tau_e = L / R
 *   T_motor  = Kt * i
 *   T_fric   = Tc * tanh(omega / omega_eps) + b * omega
 *   J * domega/dt = T_motor - T_fric - T_load - T_jam
 *   dtheta/dt     = omega
 * @endverbatim
 *
 * The electrical time constant is modelled as a first-order lag on the current
 * rather than by integrating `L di/dt = V - R i - Ke w` directly.  With
 * `L = 1.5 mH` and `R = 2 ohm` the electrical pole sits at 1/(0.75 ms), so an
 * explicit Euler step of 1 ms would be marginally unstable and would produce
 * a numerical oscillation that has nothing to do with the machine.  Collapsing
 * the fast pole into `di/dt = (i_ss - i)/tau_e` is the standard singular
 * perturbation and it keeps the *physical* time constant visible while staying
 * stable at the 1 ms control period.
 *
 * # Model (gripper axis and tea stem)
 * @verbatim
 *   v_jaw    += (v_cmd - v_jaw) * dt / (tau_jaw + dt)         (servo lag)
 *   v_jaw    -= F / K_backdrive                               (force back-drive)
 *   gap      += v_jaw * dt,  clamped to [0, gap_max]
 *   d         = max(0, stem_gap - gap)                        (penetration, mm)
 *   F         = k1 * d + k2 * d^2 + c * max(0, -v_jaw)        (N)
 * @endverbatim
 *
 * The quadratic term is what makes the stem **stiffen** as it is compressed,
 * and it is the reason a fixed-position gripper is unsafe: closing to a fixed
 * gap on a stiffer-than-expected stem raises the force much faster than a
 * linear model predicts, and the damage flag latches.
 *
 * @note Units follow the project-wide convention documented in `config.h`:
 *       mrad, mrad/s, mA, mm, N, V, N*m.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_PLANT_MODEL_H
#define TP_PLANT_MODEL_H

#include <stdint.h>

#include "status.h"
#include "compiler.h"
#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Electrical and mechanical parameters
 * ========================================================================= */

/** @brief Motor winding resistance, ohm (post-gearbox equivalent). */
#define TP_PLANT_MOTOR_R_OHM        (2.0f)
/** @brief Motor winding inductance, henry. */
#define TP_PLANT_MOTOR_L_H          (1.5e-3f)
/** @brief Electrical time constant `L/R`, seconds. */
#define TP_PLANT_MOTOR_TAU_E_S      (TP_PLANT_MOTOR_L_H / TP_PLANT_MOTOR_R_OHM)

/**
 * @name Torque and back-EMF constants
 *
 * **These two numbers must be equal.**  For any lossless electromechanical
 * energy conversion the torque constant in N*m/A and the back-EMF constant in
 * V*s/rad are the *same number*: the identity follows from
 * `P_mech = T*w = (Kt*I)*w` and `P_elec = V*I = (Ke*w)*I`, which must agree.
 * A model with `Kt != Ke` describes a machine that produces more torque per amp
 * than its own back-EMF implies, i.e. one that does not conserve energy.
 *
 * The first revision had `Kt = 2.0` and `Ke = 0.05` - a factor of 40 apart -
 * which no compiler, no test and no simulation run complained about.  It only
 * surfaced when the numbers were checked against each other rather than each
 * against the behaviour it was tuned for.  The static assertion below is what
 * makes that check permanent.
 *
 * A gearbox does not break the identity: referring both constants to the joint
 * output multiplies each by the same reduction ratio, so they stay equal.
 *
 * **Why 0.4.**  Two requirements set the range, and they pull in opposite
 * directions:
 *   - top speed: `V_bus / Ke >= 60 rad/s` needs `Ke <= 24/60 = 0.4` V*s/rad;
 *   - holding torque: the joint must hold its load at a sane current, which
 *     needs `Kt` large enough that `Kt * I_continuous` covers the load torque
 *     with margin.  At `Kt = 0.4` and the 3 A continuous limit that is
 *     1.2 N*m against a 20 mN*m modelled load, and 2.0 N*m at the 5 A peak -
 *     ample, and reachable with a 24 V bus.
 *
 * The pair `(0.4, 0.4)` is the only value satisfying both, which is why the
 * numbers are derived here rather than picked.
 * @{ */
#define TP_PLANT_MOTOR_KT_NM_PER_A  (0.4f)
#define TP_PLANT_MOTOR_KE_VS_RAD    (0.4f)
/** @} */
/** @brief Joint-space inertia, kg*m^2. */
#define TP_PLANT_JOINT_J_KGM2       (2.5e-3f)
/** @brief Viscous friction coefficient, N*m*s/rad. */
#define TP_PLANT_JOINT_B_NMS_RAD    (3.0e-5f)
/** @brief Coulomb friction torque magnitude, N*m. */
#define TP_PLANT_JOINT_TC_NM        (0.02f)
/** @brief Velocity scale of the Coulomb-friction tanh smoothing, rad/s. */
#define TP_PLANT_JOINT_OMEGA_EPS_RAD_S (0.2f)

/** @brief Gripper jaw servo time constant, seconds. */
#define TP_PLANT_JAW_TAU_S          (8.0e-3f)
/** @brief Jaw force back-drive coefficient, N*s/mm. */
#define TP_PLANT_JAW_BACKDRIVE      (20.0f)
/** @brief Nominal closed-jaw mechanical stop, mm (gap can never go below). */
#define TP_PLANT_JAW_HARD_STOP_MM   (1.0f)

/** @brief Default tea-stem surface position, mm of jaw gap at first contact. */
#define TP_PLANT_STEM_GAP_MM        (6.0f)
/** @brief Linear stem stiffness, N/mm. */
#define TP_PLANT_STEM_K1_N_PER_MM   (0.30f)
/** @brief Quadratic (stiffening) stem coefficient, N/mm^2. */
#define TP_PLANT_STEM_K2_N_PER_MM2  (0.06f)
/** @brief Stem internal damping, N*s/mm.
 *
 * This is the Kelvin-Voigt damping term of the contact model, and it is what
 * makes the force *step* at the instant of contact rather than rising smoothly
 * from zero: at the 40 mm/s fast-approach speed a coefficient of 0.05 N*s/mm
 * would add 2.0 N of force in a single sample.  That is physically real (it is
 * why impacts damage soft objects) but it is larger than a tea stem warrants,
 * so the value is set to give 0.6 N at the fast-approach speed and 0.09 N at
 * the slow-approach speed.  Getting this number wrong is also what false-tripped
 * the force-sensor spike detector in the first revision. */
#define TP_PLANT_STEM_DAMP_N_S_MM   (0.015f)
/** @brief Force above which the bud is considered damaged, N (latching). */
#define TP_PLANT_STEM_DAMAGE_N      (4.0f)

/* =========================================================================
 * Compile-time consistency checks
 * ========================================================================= */

/**
 * @brief The torque and back-EMF constants must be numerically equal.
 *
 * See the derivation above: they are the same physical quantity expressed in
 * different units, so a mismatch means the model does not conserve energy.  The
 * first revision was out by a factor of 40 and nothing caught it.
 */
TP_STATIC_ASSERT(TP_PLANT_MOTOR_KT_NM_PER_A == TP_PLANT_MOTOR_KE_VS_RAD,
                 "Kt and Ke must be equal in SI units (energy conservation)");
TP_STATIC_ASSERT(TP_PLANT_MOTOR_TAU_E_S < 0.001f,
                 "the electrical pole must be much faster than the 1 ms step");
/* =========================================================================
 * State
 * ========================================================================= */

/**
 * @brief Joint axis state.
 */
typedef struct {
    float theta_mrad;    /**< Joint angle.                                   */
    float omega_mrad_s;  /**< Joint angular velocity.                        */
    float current_a;     /**< Actual winding current.                        */
    float pwm_permille;  /**< Slew-limited applied duty.                     */
    float load_torque_nm;/**< Constant external load (gravity proxy).        */
} tp_plant_joint_t;

/**
 * @brief Gripper axis and tea-stem state.
 */
typedef struct {
    float gap_mm;         /**< Jaw gap; decreases as the gripper closes.     */
    float gap_vel_mm_s;   /**< Jaw closing velocity, negative when closing.  */
    float force_n;        /**< Modelled contact force.                       */
    float peak_force_n;   /**< Maximum force seen since the last reset.      */
    float damage_index;   /**< Integral of force above the damage threshold. */
    int   damaged;        /**< Latched once the damage threshold is crossed. */
    float stem_gap_mm;    /**< Contact gap for this stem (scene parameter).  */
} tp_plant_gripper_t;

/**
 * @brief Complete plant state plus the fault-injection knobs.
 */
typedef struct {
    tp_plant_joint_t   joint;   /**< Arm axis.                                 */
    tp_plant_gripper_t gripper; /**< Gripper axis.                             */
    float    vbus_v;            /**< Supply rail, V.                           */
    uint32_t tick_ms;           /**< Simulated time.                           */
    int      joint_jammed;      /**< Non-zero applies the stick-slip jam below.*/
    float    jam_torque_nm;     /**< Jam static friction torque, N*m.  The axis
                                 *   is held while the drive's torque stays
                                 *   below this; a value above the drive's
                                 *   capability (6 N*m at the peak current
                                 *   limit) produces a true stall. */
    int      encoder_fault;     /**< Non-zero makes the encoder read garbage.  */
    float    force_noise_sigma_n;/**< Gaussian noise sigma on the force reading.*/
    float    adc_gain_error;    /**< Multiplicative sensor error, e.g. 1.05.   */
    float    adc_zero_offset_counts; /**< Additive sensor error, counts.       */
    int      estop_asserted;    /**< Mirrors the physical E-stop loop.         */
    uint32_t noise_seed;        /**< LCG state, for reproducible noise.        */
} tp_plant_t;

/**
 * @brief Initialise the plant to its power-on state.
 *
 * The joint starts at the home angle with zero velocity and the jaws fully
 * open, matching what the real firmware sees after `SELF_CHECK`.
 *
 * @param[out] p  Plant state, zeroed and then populated.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM when @p p is NULL.
 */
tp_status_t tp_plant_init(tp_plant_t *p);

/**
 * @brief Reset the plant exactly as the `RESET` command should.
 *
 * @param[in,out] p  Plant state.
 */
void tp_plant_reset(tp_plant_t *p);

/**
 * @brief Advance the plant by @p dt_s seconds.
 *
 * @param[in,out] p             Plant state.
 * @param[in]     pwm_joint     Joint duty command, per-mille.
 * @param[in]     pwm_gripper   Gripper duty command, per-mille.  The sign sets
 *                              the jaw direction; see
 *                              @ref tp_plant_gripper_velocity_to_pwm.
 * @param[in]     dt_s          Integration step in seconds.
 */
void tp_plant_step(tp_plant_t *p, float pwm_joint, float pwm_gripper, float dt_s);

/**
 * @brief Convert a desired jaw velocity into an equivalent duty command.
 *
 * The plant integrates duty into velocity through the servo lag, so a caller
 * that wants "close at 6 mm/s" has to know the mapping.  It lives here, next
 * to the model it belongs to, rather than inside a driver.
 *
 * @param[in] v_mm_s  Desired jaw velocity, negative to close.
 * @return Duty in per-mille, saturating at the bus voltage.
 */
float tp_plant_gripper_velocity_to_pwm(float v_mm_s);

/**
 * @brief Read the joint encoder as the hardware would: quantised counts.
 *
 * @param[in] p  Plant state.
 * @return Signed encoder count, or a deliberately corrupted value when
 *         @ref tp_plant_t::encoder_fault is set.
 */
int32_t tp_plant_read_encoder_count(const tp_plant_t *p);

/**
 * @brief Read the raw force ADC counts, including quantisation and noise.
 *
 * The conversion is the exact inverse of the calibration in `config.h`:
 * counts = zero + force / N_per_count, with the injected gain and offset
 * errors applied, saturation at 0 and 4095, and Gaussian noise from the
 * internal LCG.
 *
 * @param[in,out] p  Plant state (the noise generator advances).
 * @return Raw 12-bit value, 0..4095.
 */
uint16_t tp_plant_read_force_counts(tp_plant_t *p);

/**
 * @brief Read the motor current as the shunt amplifier would: ADC counts.
 *
 * @param[in] p    Plant state.
 * @param[in] ch   Motor channel.
 * @return Raw 12-bit value, 0..4095, with 2048 counts meaning 0 A.
 */
uint16_t tp_plant_read_current_counts(const tp_plant_t *p, int ch);

/**
 * @brief Read the 24 V rail as ADC counts (10:1 divider).
 *
 * @param[in] p  Plant state.
 * @return Raw 12-bit value, 0..4095.
 */
uint16_t tp_plant_read_vbus_counts(const tp_plant_t *p);

/**
 * @brief Read the discrete-input bitmask.
 *
 * Derives limit switches from the joint angle and the jaw position so a limit
 * fault can actually be provoked in simulation.
 *
 * @param[in] p  Plant state.
 * @return Bitmask indexed by the `tp_io_id_t` enumerators.
 */
uint8_t tp_plant_read_io(const tp_plant_t *p);

/* =========================================================================
 * Fault injection, used by the test scenarios
 * ========================================================================= */

/**
 * @brief Clamp the joint with a large resisting torque.
 *
 * @param[in,out] p           Plant state.
 * @param[in]     torque_nm   Resisting torque magnitude; 0 releases the jam.
 */
void tp_plant_set_jam(tp_plant_t *p, float torque_nm);

/**
 * @brief Make the encoder report garbage, to exercise `ERR_ENCODER`.
 *
 * @param[in,out] p      Plant state.
 * @param[in]     enable Non-zero to corrupt the reading.
 */
void tp_plant_set_encoder_fault(tp_plant_t *p, int enable);

/**
 * @brief Inject zero-mean Gaussian noise on the force reading.
 *
 * @param[in,out] p      Plant state.
 * @param[in]     sigma_n Noise standard deviation in newtons; 0 disables it.
 */
void tp_plant_set_force_noise(tp_plant_t *p, float sigma_n);

/**
 * @brief Inject a multiplicative and additive sensor calibration error.
 *
 * @param[in,out] p             Plant state.
 * @param[in]     gain_error    Multiplier, 1.0 for a perfect sensor.
 * @param[in]     offset_counts Additive error in ADC counts.
 */
void tp_plant_set_sensor_error(tp_plant_t *p, float gain_error, float offset_counts);

/**
 * @brief Move the stem surface, i.e. change how far the jaws close.
 *
 * @param[in,out] p        Plant state.
 * @param[in]     gap_mm   Jaw gap at which contact begins.
 */
void tp_plant_set_stem_gap(tp_plant_t *p, float gap_mm);

/**
 * @brief Seed the noise generator so a run is reproducible.
 *
 * @param[in,out] p     Plant state.
 * @param[in]     seed  Non-zero seed; 0 is replaced by 1 to avoid the
 *                      degenerate all-zero state of the LCG.
 */
void tp_plant_set_seed(tp_plant_t *p, uint32_t seed);

#ifdef __cplusplus
}
#endif

#endif /* TP_PLANT_MODEL_H */
