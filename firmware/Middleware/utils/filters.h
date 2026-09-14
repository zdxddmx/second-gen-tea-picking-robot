/**
 * @file    filters.h
 * @brief   Small, allocation-free digital filters for sensor conditioning.
 *
 * @details
 * # Why these three filters and not one
 *
 * The force channel has three distinct noise problems, and no single filter
 * solves all of them:
 *
 * 1. **Broadband / quantisation noise.**  A 12-bit ADC at 500 Hz on a
 *    1 mV/V load cell gives an LSB of roughly 12 mN, so the raw signal is
 *    visibly steppy.  A **moving average** attenuates it well and, importantly
 *    for a robot, is *linear phase*: it delays every frequency equally, so it
 *    does not distort the shape of the contact transient.
 * 2. **High-frequency switching noise from the motor drive.**  The PWM
 *    carrier couples into the load-cell cabling.  A **first-order low-pass**
 *    gives a predictable, cheap -20 dB/decade roll-off and — unlike the moving
 *    average — has only one state variable, which makes it trivial to reason
 *    about in a 1 kHz loop.
 * 3. **Isolated spikes.**  A single corrupted ADC conversion (a DMA overrun,
 *    an EMI burst) produces a one-sample outlier.  Both of the filters above
 *    *spread* that spike over their whole window instead of removing it.  A
 *    **median** filter removes it completely, which is why it runs first.
 *
 * Recommended chain (and the one used by @ref force_sensor.c):
 *
 * @verbatim
 *   raw -> median(5) -> moving_average(8) -> low_pass(tau = 6 ms) -> filtered
 * @endverbatim
 *
 * # What happens if you skip them
 *
 * - Without the median: one bad sample becomes a visible force glitch of
 *   hundreds of millinewtons, which the force loop reads as a contact event.
 * - Without the moving average: the derivative term in the force PID
 *   amplifies LSB quantisation into PWM jitter, heating the gripper servo for
 *   no useful motion.
 * - Without the low pass: the 20 kHz PWM carrier aliases down into the control
 *   band and is indistinguishable from real force.
 *
 * All filters here are:
 *   - **allocation-free** (state lives inside the struct, fixed-size buffers),
 *   - **reset-safe** (`_reset` clears history so a mode change cannot blend
 *     two unrelated signals),
 *   - **reentrant per instance**, so a second force channel needs no new code.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_FILTERS_H
#define TP_FILTERS_H

#include <stdint.h>

#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum window length supported by the median filter. */
#define TP_FILTER_MEDIAN_MAX 9u

/**
 * @brief Fixed-capacity circular moving-average filter.
 *
 * O(1) per sample: the running sum is updated incrementally rather than
 * re-accumulated, which matters at 500 Hz on a Cortex-M4 with no FPU double
 * support.
 */
typedef struct {
    float    buffer[TP_FILTER_MEDIAN_MAX]; /**< History, ring-indexed.        */
    uint32_t window;                       /**< Active window length, >= 1.  */
    uint32_t index;                        /**< Next write position.         */
    uint32_t count;                        /**< Samples seen so far.         */
    float    sum;                          /**< Running sum of @ref buffer.  */
    float    value;                        /**< Latest output.               */
} tp_filter_mavg_t;

/**
 * @brief Initialise a moving-average filter.
 *
 * @param[out] f       Filter instance.
 * @param[in]  window  Window length in samples, clamped to `[1, 9]`.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM if @p f is NULL.
 *
 * @note The filter is primed so that the first output equals the first input
 *       instead of ramping up from zero.  A ramp-up would look like a slow
 *       contact event to the gripper controller.
 */
tp_status_t tp_filter_mavg_init(tp_filter_mavg_t *f, uint32_t window);

/**
 * @brief Push one sample.
 *
 * @param[in,out] f   Filter instance.
 * @param[in]     x   New sample.  Non-finite values are ignored and the
 *                    previous output is returned.
 * @return Filtered value.
 */
float tp_filter_mavg_update(tp_filter_mavg_t *f, float x);

/**
 * @brief Clear the filter history and re-arm priming.
 * @param[in,out] f  Filter instance.
 */
void tp_filter_mavg_reset(tp_filter_mavg_t *f);

/**
 * @brief Latest output without pushing a sample.
 * @param[in] f  Filter instance, may be NULL.
 * @return Last output, or 0.0f.
 */
float tp_filter_mavg_value(const tp_filter_mavg_t *f);

/**
 * @brief First-order IIR low-pass, `y += alpha * (x - y)`.
 *
 * The coefficient is derived from a physical time constant rather than being
 * entered directly, because "tau = 6 ms" is reviewable and "alpha = 0.14" is
 * not.  Discretisation is backward Euler, so the filter stays stable for any
 * `dt`, including `dt > tau`.
 */
typedef struct {
    float tau_s;   /**< Time constant in seconds; 0 disables filtering.     */
    float dt_s;    /**< Sample time in seconds.                             */
    float alpha;   /**< Precomputed `dt / (tau + dt)`.                      */
    float value;   /**< Latest output.                                      */
    int   primed;  /**< Non-zero once the first sample has been absorbed.   */
} tp_filter_lpf_t;

/**
 * @brief Initialise a first-order low-pass filter.
 *
 * @param[out] f      Filter instance.
 * @param[in]  tau_s  Time constant in seconds; 0 means "pass through".
 * @param[in]  dt_s   Sample time in seconds, must be > 0.
 * @return @ref TP_OK, @ref TP_ERR_PARAM or @ref TP_ERR_RANGE.
 */
tp_status_t tp_filter_lpf_init(tp_filter_lpf_t *f, float tau_s, float dt_s);

/**
 * @brief Change the time constant at runtime (keeps the current state).
 *
 * @param[in,out] f      Filter instance.
 * @param[in]     tau_s  New time constant in seconds.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_filter_lpf_set_tau(tp_filter_lpf_t *f, float tau_s);

/**
 * @brief Push one sample.
 *
 * @param[in,out] f  Filter instance.
 * @param[in]     x  New sample.
 * @return Filtered value.
 */
float tp_filter_lpf_update(tp_filter_lpf_t *f, float x);

/**
 * @brief Clear the filter state.
 * @param[in,out] f  Filter instance.
 */
void tp_filter_lpf_reset(tp_filter_lpf_t *f);

/**
 * @brief Latest output without pushing a sample.
 * @param[in] f  Filter instance, may be NULL.
 * @return Last output, or 0.0f.
 */
float tp_filter_lpf_value(const tp_filter_lpf_t *f);

/**
 * @brief Sliding median over an odd-length window.
 *
 * Implemented with an insertion sort over a small fixed window: at a window
 * of 5 that is at most 10 comparisons, far cheaper than any heap-based
 * approach and with no allocation.
 */
typedef struct {
    float    window[TP_FILTER_MEDIAN_MAX]; /**< Sliding history.             */
    uint32_t length;                       /**< Active window, odd, 1..9.    */
    uint32_t index;                        /**< Next write position.         */
    uint32_t count;                        /**< Samples seen so far.         */
    float    value;                        /**< Latest median.               */
} tp_filter_median_t;

/**
 * @brief Initialise a median filter.
 *
 * @param[out] f       Filter instance.
 * @param[in]  length  Window length; even values are rounded up to the next
 *                     odd number so a true median always exists.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_filter_median_init(tp_filter_median_t *f, uint32_t length);

/**
 * @brief Push one sample and return the current median.
 *
 * @param[in,out] f  Filter instance.
 * @param[in]     x  New sample.
 * @return Median of the samples currently in the window.
 */
float tp_filter_median_update(tp_filter_median_t *f, float x);

/**
 * @brief Clear the median filter history.
 * @param[in,out] f  Filter instance.
 */
void tp_filter_median_reset(tp_filter_median_t *f);

/**
 * @brief Slew-rate limiter.
 *
 * Used to keep the demanded motor voltage from stepping, which would otherwise
 * produce a current spike limited only by the winding resistance.  The clamp is
 * expressed in units per second so it stays meaningful if the task rate changes.
 */
typedef struct {
    float rate_per_s; /**< Maximum |dy/dt|.                                  */
    float dt_s;       /**< Sample time.                                      */
    float value;      /**< Latest output.                                    */
    int   primed;     /**< Non-zero once the first sample has been absorbed. */
} tp_filter_slew_t;

/**
 * @brief Initialise a slew-rate limiter.
 *
 * @param[out] s           Limiter instance.
 * @param[in]  rate_per_s  Maximum rate of change, units per second.
 * @param[in]  dt_s        Sample time in seconds.
 * @return @ref TP_OK, @ref TP_ERR_PARAM or @ref TP_ERR_RANGE.
 */
tp_status_t tp_filter_slew_init(tp_filter_slew_t *s, float rate_per_s, float dt_s);

/**
 * @brief Push one sample through the limiter.
 *
 * @param[in,out] s  Limiter instance.
 * @param[in]     x  Target value.
 * @return Rate-limited value.
 */
float tp_filter_slew_update(tp_filter_slew_t *s, float x);

/**
 * @brief Clear the limiter (the next sample passes through unmodified).
 * @param[in,out] s  Limiter instance.
 */
void tp_filter_slew_reset(tp_filter_slew_t *s);

/**
 * @brief Spike detector based on a deviation ratio.
 *
 * Counts how often `|x - y_prev|` exceeds @ref tp_spike_detector_t::threshold.
 * The force sensor uses it to raise `ERR_SENSOR` when the analogue front end
 * starts producing jumps that no physical contact could explain.
 */
typedef struct {
    float    threshold;  /**< Absolute jump considered implausible.         */
    float    prev;       /**< Previous accepted sample.                     */
    uint32_t spikes;     /**< Total spikes observed.                        */
    uint32_t streak;     /**< Consecutive spikes; reset by a good sample.   */
    int      primed;     /**< Non-zero once a first sample has been seen.   */
} tp_spike_detector_t;

/**
 * @brief Initialise a spike detector.
 *
 * @param[out] d          Detector instance.
 * @param[in]  threshold  Maximum plausible sample-to-sample jump.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_spike_detector_init(tp_spike_detector_t *d, float threshold);

/**
 * @brief Feed one sample to the detector.
 *
 * @param[in,out] d  Detector instance.
 * @param[in]     x  New sample.
 * @return 1 when this sample was a spike (and was therefore *not* stored as
 *         the new reference), 0 otherwise.
 */
int tp_spike_detector_update(tp_spike_detector_t *d, float x);

/**
 * @brief Clear the detector statistics.
 * @param[in,out] d  Detector instance.
 */
void tp_spike_detector_reset(tp_spike_detector_t *d);

#ifdef __cplusplus
}
#endif

#endif /* TP_FILTERS_H */
