/**
 * @file    pid.h
 * @brief   Production-grade PID controller used by every loop in the robot.
 *
 * @details
 * # What it is
 * A single, well-tested PID implementation that is instantiated three times:
 * the joint **position** loop, the joint **velocity** loop and the gripper
 * **force** loop.  It is deliberately not a class hierarchy or a template: on
 * a Cortex-M4 a plain struct plus free functions keeps the code in flash, the
 * state in `.bss` and the behaviour fully visible in a debugger.
 *
 * # Why the extra features (and what breaks without them)
 *
 * | Feature | Why it exists | What happens without it |
 * |---|---|---|
 * | Integral clamp | Bounds the contribution of the I term so a long approach phase cannot charge the integrator to a huge value | The joint overshoots badly on the first move after a long standby |
 * | Anti-windup (back-calculation) | While the PWM is saturated the actuator cannot follow the controller, so integrating the error is pure fiction | Classic "windup": the arm slams into the target, then takes seconds to come back |
 * | Conditional integration | Cheaper alternative that freezes the integrator exactly while saturation persists in the same direction | Same as above, but implemented as a branch instead of a feedback term |
 * | Derivative on **measurement** | Differentiating the error creates an impulse whenever the setpoint steps, which the motor happily turns into a current spike | "Derivative kick": audible clunk and a visible current spike on every new target |
 * | Derivative low-pass | The encoder is quantised; a raw difference at 1 kHz is mostly quantisation noise, and D amplifies it | The PWM buzzes and the motor heats up for no control benefit |
 * | Trapezoidal integration | Rectangular integration of a moving error biases the I term, which shows up as a steady-state offset | Position error settles at a small but permanent non-zero value |
 * | Runtime gain retune | The host must be able to change gains over the link without a reflash (needed for Experiment 02) | Gain scheduling requires a firmware rebuild per data point |
 * | `tp_pid_reset()` | A mode change (approach -> force control) must not inherit a stale integrator | A leftover I term from the previous phase injects an unexplained bias |
 *
 * # Discrete form actually implemented
 *
 * @verbatim
 *   e[k]      = r[k] - y[k]
 *   P[k]      = Kp * e[k]
 *   I[k]      = I[k-1] + Ki * 0.5 * (e[k] + e[k-1]) * dt        (trapezoidal)
 *   I[k]      = clamp(I[k], -I_max, +I_max)
 *   d_raw[k]  = -(y[k] - y[k-1]) / dt                            (on measurement)
 *   D[k]      = D[k-1] + (d_raw[k] - D[k-1]) * dt / (tau_d + dt)  (1st-order LPF)
 *   u_raw[k]  = P[k] + I[k] + Kd * D[k] + Kff * ff[k]
 *   u[k]      = clamp(u_raw[k], u_min, u_max)
 *   I[k]     += Tt_gain * (u[k] - u_raw[k]) * dt                  (back-calculation)
 * @endverbatim
 *
 * Note that `Ki`, `Kd` and `Kff` here are *not* the classical parallel-form
 * gains: the integral term is already multiplied by `dt` inside the update, so
 * `Ki` is expressed in output-units per (error-unit x second) and can be moved
 * to a different `dt` without changing the closed-loop behaviour.  This is the
 * "velocity form" convention and it is what makes runtime `dt` changes safe.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_PID_H
#define TP_PID_H

#include <stdint.h>

#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Anti-windup strategy.
 */
typedef enum {
    /** No protection.  Only acceptable for a pure P or D controller. */
    TP_PID_AW_NONE = 0,
    /**
     * Freeze the integrator while the output is saturated *and* the error is
     * pushing further into the same saturation.  Cheap: one comparison.
     */
    TP_PID_AW_CLAMP_INTEGRAL = 1,
    /**
     * Back-calculation: feed the saturation error `(u_sat - u_unsat)` back
     * into the integrator through a tracking gain.  Smooth, and it unwinds the
     * integrator automatically when the actuator leaves saturation.
     */
    TP_PID_AW_BACK_CALCULATION = 2
} tp_pid_antiwindup_t;

/**
 * @brief Immutable-ish configuration handed to @ref tp_pid_init.
 *
 * A struct is used instead of a 12-argument constructor so that adding a
 * parameter later does not break every call site, and so a test can build a
 * configuration by naming only the fields it cares about.
 */
typedef struct {
    float kp;                       /**< Proportional gain.                   */
    float ki;                       /**< Integral gain, per second.           */
    float kd;                       /**< Derivative gain.                     */
    float kff;                      /**< Feed-forward gain, 0 disables it.    */
    float dt_s;                     /**< Sample time in seconds, > 0.         */
    float out_min;                  /**< Lower output clamp.                  */
    float out_max;                  /**< Upper output clamp.                  */
    float i_min;                    /**< Lower integral clamp (output units). */
    float i_max;                    /**< Upper integral clamp (output units). */
    float tau_d_s;                  /**< Derivative LPF time constant; 0 = off.*/
    float tau_i_s;                  /**< Integral leak time constant; 0 = off. */
    float aw_gain;                  /**< Tracking gain `1/Tt`; <= 0 = auto.   */
    int   derivative_on_measurement;/**< Non-zero to differentiate `y`, not `e`.*/
    tp_pid_antiwindup_t antiwindup; /**< Strategy selector.                   */
} tp_pid_config_t;

/**
 * @brief Per-term breakdown, for telemetry and for the Python cross-check.
 */
typedef struct {
    float p;          /**< Proportional contribution.                      */
    float i;          /**< Integral contribution.                          */
    float d;          /**< Derivative contribution, already gain-scaled.   */
    float ff;         /**< Feed-forward contribution.                      */
    float out;        /**< Final, clamped output.                          */
    float out_raw;    /**< Pre-clamp output.                               */
    float error;      /**< Last error `r - y`.                             */
    float derivative; /**< Last filtered derivative of the measurement.    */
    int   saturated;  /**< +1 upper clamp, -1 lower clamp, 0 free.         */
} tp_pid_terms_t;

/**
 * @brief PID controller instance.
 *
 * Copying an instance with `=` is supported and is how the simulator snapshots
 * controller state; it contains no pointers.
 */
typedef struct {
    /* --- configuration ------------------------------------------------- */
    float kp, ki, kd, kff;
    float dt_s;
    float out_min, out_max;
    float i_min, i_max;
    float tau_d_s;
    float tau_i_s;
    float aw_gain;
    int   derivative_on_measurement;
    tp_pid_antiwindup_t antiwindup;

    /* --- state ---------------------------------------------------------- */
    float    integral;      /**< Accumulated integral term.                 */
    float    err_prev;      /**< `e[k-1]` for trapezoidal integration.      */
    float    meas_prev;     /**< `y[k-1]` for derivative on measurement.    */
    float    d_filt;        /**< Filtered derivative state.                 */
    float    last_output;   /**< Held and returned if a fault is detected.  */
    int      initialised;   /**< Non-zero once @ref tp_pid_init succeeded.  */
    int      first_update;  /**< Non-zero until the first sample is taken.  */
    int      diverged;      /**< Latched if a non-finite value was seen.    */
    tp_pid_terms_t terms;   /**< Last computed term breakdown.              */
    uint32_t update_count;  /**< Number of @ref tp_pid_update calls.        */
} tp_pid_t;

/**
 * @brief Initialise a PID instance.
 *
 * @param[out] pid  Instance to initialise; zeroed by the call.
 * @param[in]  cfg  Configuration.  Must not be NULL.  Invalid fields (NULL
 *                  config, `dt_s <= 0`, inverted output limits) are rejected
 *                  rather than silently repaired, so a wiring mistake in
 *                  `config.h` shows up as a failed init at boot.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM on a rejected configuration.
 */
tp_status_t tp_pid_init(tp_pid_t *pid, const tp_pid_config_t *cfg);

/**
 * @brief Change the gains at runtime.
 *
 * The integrator and derivative state are preserved, so gain scheduling does
 * not create a transient.  Intended for the `SET_PID` protocol command and for
 * Experiment 02 (parameter sweep without reflashing).
 *
 * @param[in,out] pid  Instance.
 * @param[in]     kp   Proportional gain.
 * @param[in]     ki   Integral gain.
 * @param[in]     kd   Derivative gain.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM if @p pid is NULL / uninitialised.
 */
tp_status_t tp_pid_set_gains(tp_pid_t *pid, float kp, float ki, float kd);

/**
 * @brief Change the feed-forward gain at runtime.
 *
 * @param[in,out] pid  Instance.
 * @param[in]     kff  Feed-forward gain; 0 disables the feed-forward path.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_pid_set_ff_gain(tp_pid_t *pid, float kff);

/**
 * @brief Change the output and integral clamps at runtime.
 *
 * @param[in,out] pid      Instance.
 * @param[in]     out_min  Lower output clamp.
 * @param[in]     out_max  Upper output clamp, must be > @p out_min.
 * @param[in]     i_min    Lower integral clamp.
 * @param[in]     i_max    Upper integral clamp, must be > @p i_min.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_pid_set_limits(tp_pid_t *pid, float out_min, float out_max,
                              float i_min, float i_max);

/**
 * @brief Change the sample time, e.g. if a task rate is reconfigured.
 *
 * @param[in,out] pid   Instance.
 * @param[in]     dt_s  New sample time in seconds, must be > 0 and finite.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_pid_set_dt(tp_pid_t *pid, float dt_s);

/**
 * @brief Clear the integrator, derivative memory and the divergence latch.
 *
 * Must be called on every control-mode change (for example when the gripper
 * hands over from approach to force control).  The gains and limits are kept.
 *
 * @param[in,out] pid  Instance.  A NULL pointer is ignored.
 */
void tp_pid_reset(tp_pid_t *pid);

/**
 * @brief Run one controller update without feed-forward.
 *
 * @param[in,out] pid        Instance.
 * @param[in]     setpoint   Reference `r[k]`.
 * @param[in]     measurement Feedback `y[k]`.
 * @return Clamped control output.  If @p pid is NULL, uninitialised or has
 *         diverged, the last known good output is returned unchanged.
 */
float tp_pid_update(tp_pid_t *pid, float setpoint, float measurement);

/**
 * @brief Run one controller update with a feed-forward term.
 *
 * @param[in,out] pid          Instance.
 * @param[in]     setpoint     Reference `r[k]`.
 * @param[in]     measurement  Feedback `y[k]`.
 * @param[in]     feedforward  Feed-forward signal, scaled by `kff`.
 *                             Use 0.0f for none.
 * @return Clamped control output.
 */
float tp_pid_update_ff(tp_pid_t *pid, float setpoint, float measurement,
                       float feedforward);

/**
 * @brief Copy out the last computed term breakdown.
 *
 * @param[in]  pid    Instance.
 * @param[out] terms  Receives the breakdown.  Zeroed when @p pid is NULL.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_pid_get_terms(const tp_pid_t *pid, tp_pid_terms_t *terms);

/**
 * @brief Whether the controller has detected a non-finite intermediate value.
 *
 * Once latched, @ref tp_pid_update returns the last good output and the
 * safety layer raises `ERR_ENCODER`/`ERR_SENSOR` depending on the loop.  Only
 * @ref tp_pid_reset clears it.
 *
 * @param[in] pid  Instance.
 * @return Non-zero when diverged.
 */
int tp_pid_is_diverged(const tp_pid_t *pid);

/**
 * @brief Build a default configuration from the `config.h` macros.
 *
 * Provided as a convenience so that the three real loops are configured from
 * one place and a test can start from a known, sane baseline.
 *
 * @param[in] kp  Proportional gain.
 * @param[in] ki  Integral gain.
 * @param[in] kd  Derivative gain.
 * @param[in] dt_s Sample time.
 * @param[in] out_max Symmetric output clamp magnitude.
 * @param[in] i_max Symmetric integral clamp magnitude.
 * @param[in] tau_d_s Derivative filter time constant.
 * @return A fully populated configuration using back-calculation anti-windup
 *         and derivative-on-measurement.
 */
tp_pid_config_t tp_pid_config_default(float kp, float ki, float kd, float dt_s,
                                      float out_max, float i_max, float tau_d_s);

#ifdef __cplusplus
}
#endif

#endif /* TP_PID_H */
