/**
 * @file    force_sensor.h
 * @brief   Load-cell force channel: acquisition, filtering, calibration, health.
 *
 * @details
 * # Signal chain implemented here
 * @verbatim
 *   Load cell -> INA333 (G=201) -> anti-alias RC -> ADC1_IN0 (12 bit, DMA)
 *        |
 *        v
 *   [ raw counts ]
 *        |
 *        +--> spike detector (median-like outlier rejection, feeds ERR_SENSOR)
 *        |
 *        +--> median(5) --> moving average(8) --> low-pass(tau = 6 ms)
 *        |                                             |
 *        |                                             v
 *        +--------------------------------------> [ N ]  calibrated, filtered
 *                                                      |
 *                                                      v
 *                                            force_control.c, safety_monitor.c
 * @endverbatim
 *
 * # Calibration
 * Two parameters fully describe the channel: an **offset** (the count value at
 * zero load) and a **scale** (newtons per count).  Both are stored in this
 * struct and both can be re-derived at runtime:
 *
 *   - @ref tp_force_sensor_zero_calibrate averages N samples with the gripper
 *     open and no bud present, and stores the result as the offset.  It is the
 *     routine a user runs from the monitor tool before a picking session.
 *   - @ref tp_force_sensor_set_scale takes a known mass hung on the jaws and
 *     solves for the scale.  The procedure, the sources of error and the
 *     acceptance criteria are in `docs/force_sensor_calibration.md`.
 *
 * `config.h` supplies the *design* values as defaults, so the robot is usable
 * before it has ever been calibrated — it is just less accurate, and the code
 * says so by leaving @ref tp_force_sensor_t::calibrated at zero.
 *
 * # Why the offset needs re-doing at all
 * The bridge's zero output drifts with temperature (the gauge factor and the
 * bridge resistances are both temperature dependent).  A 5 ppm/K mismatch on a
 * 1005 mV full-scale signal is ~5 uV/K, which over a 20 K swing is about
 * 0.1 mV -> 0.13 N.  That is 6 % of the 2.2 N target, i.e. larger than the
 * control loop's steady-state error budget, so a zero calibration at the start
 * of each session is not optional.
 *
 * # Why the health flags matter
 * A load cell fails in ways that look like real force: a cracked gauge gives a
 * large offset drift, a broken excitation wire gives a rail-to-rail reading,
 * and EMI gives spikes.  @ref tp_force_sensor_t::fault is set when any of these
 * are detected, and `force_control.c` responds by refusing to close the jaws on
 * force feedback.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_FORCE_SENSOR_H
#define TP_FORCE_SENSOR_H

#include <stdint.h>

#include "bsp_types.h"
#include "filters.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Number of samples averaged by @ref tp_force_sensor_zero_calibrate. */
#define TP_FORCE_ZERO_CAL_SAMPLES 64u

/** @brief Maximum acceptable peak-to-peak spread during zero calibration, N. */
#define TP_FORCE_ZERO_CAL_MAX_SPREAD_N 0.25f

/**
 * @brief One force channel.
 */
typedef struct {
    /* --- calibration -------------------------------------------------- */
    float adc_zero_counts;  /**< Raw count value that means 0 N.            */
    float n_per_count;      /**< Scale: newtons per ADC count.              */
    int   calibrated;       /**< Non-zero once a real calibration has run.  */

    /* --- raw and intermediate values ---------------------------------- */
    uint16_t adc_counts;    /**< Latest raw conversion.                     */
    float    force_raw_n;   /**< Calibrated but unfiltered.                 */
    float    force_n;       /**< Final filtered output, fed to the loops.   */

    /* --- filter chain -------------------------------------------------- */
    tp_filter_median_t   median;   /**< Stage 1: outlier removal.           */
    tp_filter_mavg_t     mavg;     /**< Stage 2: linear-phase averaging.    */
    tp_filter_lpf_t      lpf;      /**< Stage 3: carrier suppression.       */
    tp_spike_detector_t  spikes;   /**< Health monitor, runs on raw counts. */

    /* --- statistics and health ---------------------------------------- */
    float    min_n;         /**< Minimum filtered force since reset.        */
    float    max_n;         /**< Maximum filtered force since reset.        */
    uint32_t sample_count;  /**< Samples processed.                         */
    uint32_t rejected_count;/**< Samples discarded as non-finite.           */
    uint32_t fault_count;   /**< Times the fault condition was entered.     */
    uint32_t last_zero_cal_ms;/**< Timestamp of the last zero calibration.  */
    float    last_zero_spread_n;/**< Spread observed during that calibration.*/
    int      fault;         /**< Non-zero when the channel is untrustworthy.*/
    int      saturated;     /**< Non-zero when the ADC hit 0 or 4095.       */
    int      initialised;   /**< Non-zero after a successful init.          */
} tp_force_sensor_t;

/**
 * @brief Initialise the force channel.
 *
 * Builds the filter chain from `config.h` and loads the design calibration so
 * the channel produces usable newtons immediately.
 *
 * @param[in,out] fs  Instance.
 * @param[in]     dt_s Sensor task period in seconds; sets the low-pass alpha.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_force_sensor_init(tp_force_sensor_t *fs, float dt_s);

/**
 * @brief Feed one raw ADC conversion through the whole chain.
 *
 * @param[in,out] fs      Instance.
 * @param[in]     counts  Raw 12-bit value.
 * @return @ref TP_OK, or @ref TP_ERR_NOT_READY when @p fs is not initialised.
 *
 * @note The spike detector runs on the **raw** value, before the median.  If it
 *       ran on the filtered output it could never see a spike: the whole point
 *       of the filter is to hide them.
 */
tp_status_t tp_force_sensor_update(tp_force_sensor_t *fs, uint16_t counts);

/**
 * @brief Convert raw counts to newtons with the current calibration.
 *
 * Exposed so the calibration routine and a unit test can use exactly the same
 * conversion as the run-time path, rather than reimplementing it.
 *
 * @param[in] fs      Instance.
 * @param[in] counts  Raw value.
 * @return Force in newtons.
 */
float tp_force_sensor_counts_to_n(const tp_force_sensor_t *fs, uint16_t counts);

/**
 * @brief Filtered force.
 *
 * @param[in] fs  Instance, may be NULL.
 * @return Force in newtons, or 0.0f when @p fs is NULL.
 */
float tp_force_sensor_get_force_n(const tp_force_sensor_t *fs);

/**
 * @brief Calibrated but unfiltered force, for diagnostics.
 *
 * The difference between this and @ref tp_force_sensor_get_force_n is the
 * filter chain's contribution; plotting both is how Experiment 05
 * (`docs/11_experiment_design.md`) shows what the filtering buys.
 *
 * @param[in] fs  Instance.
 * @return Force in newtons.
 */
float tp_force_sensor_get_force_raw_n(const tp_force_sensor_t *fs);

/**
 * @brief Latest raw ADC value.
 *
 * @param[in] fs  Instance.
 * @return Raw 12-bit value.
 */
uint16_t tp_force_sensor_get_counts(const tp_force_sensor_t *fs);

/**
 * @brief Re-derive the zero offset by averaging samples.
 *
 * The caller is expected to have the jaws open and nothing between them.
 * The routine reads @ref TP_FORCE_ZERO_CAL_SAMPLES conversions through the
 * platform seam, and **rejects the calibration** if the spread exceeds
 * @ref TP_FORCE_ZERO_CAL_MAX_SPREAD_N, because averaging a moving or loaded
 * signal into the offset bakes in an error that every later reading inherits.
 *
 * @param[in,out] fs  Instance.
 * @return @ref TP_OK on success; @ref TP_ERR_STATE when the signal is too
 *         noisy or moving to calibrate, @ref TP_ERR_PARAM otherwise.
 */
tp_status_t tp_force_sensor_zero_calibrate(tp_force_sensor_t *fs);

/**
 * @brief Set the scale from a known applied force.
 *
 * @param[in,out] fs           Instance.
 * @param[in]     known_force_n Known load currently applied, in newtons.
 *                              Must be non-zero; the sign sets the direction.
 * @return @ref TP_OK, or @ref TP_ERR_RANGE when @p known_force_n is zero or the
 *         resulting scale is not physically plausible.
 */
tp_status_t tp_force_sensor_calibrate_scale(tp_force_sensor_t *fs,
                                            float known_force_n);

/**
 * @brief Override the scale directly (used by a host-side calibration file).
 *
 * @param[in,out] fs          Instance.
 * @param[in]     n_per_count Newtons per ADC count; must be positive and
 *                            within a factor of ten of the design value, which
 *                            catches a units mistake such as entering
 *                            counts-per-newton instead.
 * @return @ref TP_OK or @ref TP_ERR_RANGE.
 */
tp_status_t tp_force_sensor_set_scale(tp_force_sensor_t *fs, float n_per_count);

/**
 * @brief Override the zero offset directly.
 *
 * @param[in,out] fs          Instance.
 * @param[in]     zero_counts Count value that means 0 N; must lie inside the
 *                            ADC range.
 * @return @ref TP_OK or @ref TP_ERR_RANGE.
 */
tp_status_t tp_force_sensor_set_zero(tp_force_sensor_t *fs, float zero_counts);

/**
 * @brief Clear the filter history and the statistics.
 *
 * Called when the gripper opens, so the next closure starts from a clean
 * baseline instead of the residual force of the previous bud.
 *
 * @param[in,out] fs  Instance.
 */
void tp_force_sensor_reset_tracking(tp_force_sensor_t *fs);

/**
 * @brief Whether the channel is currently untrustworthy.
 *
 * @param[in] fs  Instance.
 * @return Non-zero when the fault latch is set.
 */
int tp_force_sensor_is_faulty(const tp_force_sensor_t *fs);

/**
 * @brief Clear the fault latch if the readings are plausible again.
 *
 * @param[in,out] fs  Instance.
 * @return @ref TP_OK, or @ref TP_ERR_FAULT_ACTIVE when the raw value is still
 *         outside the plausible window.
 */
tp_status_t tp_force_sensor_clear_fault(tp_force_sensor_t *fs);

/**
 * @brief Number of spikes seen since the last reset.
 *
 * @param[in] fs  Instance.
 * @return Spike count.
 */
uint32_t tp_force_sensor_spike_count(const tp_force_sensor_t *fs);

#ifdef __cplusplus
}
#endif

#endif /* TP_FORCE_SENSOR_H */
