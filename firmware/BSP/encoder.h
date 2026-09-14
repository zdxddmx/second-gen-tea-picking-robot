/**
 * @file    encoder.h
 * @brief   Incremental quadrature encoder driver: position and velocity.
 *
 * @details
 * # What it is
 * Wraps `tp_platform_encoder_read()` (a 32-bit hardware counter) into physical
 * units and, more importantly, into a usable **velocity estimate**, which is
 * what the inner control loop actually consumes.
 *
 * # Why the velocity estimator is the hard part
 * A quadrature encoder measures *position*, not velocity, and it measures it in
 * quanta.  With 4096 counts/rev one count is 1.534 mrad.  At the 500 Hz sensor
 * rate that gives:
 *
 * | Joint speed | Counts per 2 ms sample |
 * |---|---|
 * | 60 000 mrad/s (max) | 78.2 |
 * | 6 000 mrad/s | 7.8 |
 * | 300 mrad/s | 0.39 |
 * | 30 mrad/s (stall threshold) | 0.039 |
 *
 * A naive "delta over one sample period" estimator therefore reads **exactly
 * zero** at any speed below roughly 770 mrad/s, which is precisely the region
 * where the stall detector has to work.  It would also make the velocity loop
 * see a staircase, and its derivative term would fire on every step.
 *
 * # The estimator implemented here
 * A **fixed-position-increment (constant-elapsed-position) estimator**, the
 * practical middle ground between the classic M and T methods:
 *
 *   - Accumulate counts and elapsed time since the last published estimate.
 *   - Publish a new estimate as soon as `|accumulated counts| >= MIN_INC`
 *     **or** the accumulated time reaches `MAX_WINDOW_MS`.
 *   - Between publications, hold the previous value (a zero-order hold), which
 *     is exactly what a real speed observer with a slower update rate does.
 *
 * Consequences, all of them deliberate:
 *   - High speed -> a fresh estimate every sample, full bandwidth.
 *   - Low speed -> a longer window, so quantisation error shrinks in proportion
 *     to the window instead of dominating.
 *   - Stopped -> the window expires and the estimate reads zero, which is the
 *     correct answer.
 *
 * On top of that a moving average and a first-order low-pass clean up the
 * residual stair-step.  Their registers and time constants come from
 * `config.h` so the tuning is auditable.
 *
 * # What happens without it
 * Feeding raw per-sample deltas to the velocity loop gives a controller whose
 * effective gain changes by a factor of 200 between low and high speed, plus a
 * PWM buzz from the D term.  Interviewees ask about this constantly; the table
 * above is the answer.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_ENCODER_H
#define TP_ENCODER_H

#include <stdint.h>

#include "bsp_types.h"
#include "filters.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Minimum accumulated counts before a velocity estimate is published. */
#define TP_ENCODER_VEL_MIN_INC 4

/** @brief Maximum accumulation window before an estimate is forced, ms. */
#define TP_ENCODER_VEL_MAX_WINDOW_MS 20u

/**
 * @brief One encoder channel.
 */
typedef struct {
    tp_encoder_ch_t ch;              /**< Hardware channel.                  */
    float    mrad_per_count;         /**< Scale factor.                      */
    int32_t  counts_per_rev;         /**< Counts per revolution, X4 decoded. */

    int32_t  count;                  /**< Latest absolute count.             */
    int32_t  count_prev;             /**< Count at the previous update.      */
    int32_t  delta;                  /**< Counts since the previous update.  */
    int32_t  accum_counts;           /**< Counts since the last estimate.    */
    uint32_t accum_us;               /**< Microseconds since last estimate.  */

    float    position_mrad;          /**< Absolute joint angle.              */
    float    position_offset_mrad;   /**< Value subtracted on a zero-set.    */
    float    velocity_raw_mrad_s;    /**< Estimator output before filtering. */
    float    velocity_mrad_s;        /**< Filtered output fed to the loop.   */

    tp_filter_mavg_t speed_mavg;     /**< Moving-average stage.              */
    tp_filter_lpf_t  speed_lpf;      /**< First-order low-pass stage.        */

    uint32_t last_tick_ms;           /**< Timestamp of the previous update.  */
    uint32_t sample_count;           /**< Number of @ref tp_encoder_update.  */
    uint32_t read_errors;            /**< Failed hardware reads.             */
    uint32_t direction_reversals;    /**< Counts sign flips, a wear proxy.   */
    int      last_direction;         /**< -1, 0 or +1.                       */
    int      initialised;            /**< Non-zero after a successful init.  */
    int      fault;                  /**< Non-zero when reads keep failing.  */
} tp_encoder_t;

/**
 * @brief Initialise an encoder channel.
 *
 * Reads the hardware once to seed the counter, so the first velocity estimate
 * is meaningful instead of being computed against a zero origin.
 *
 * @param[in,out] enc  Instance to initialise.
 * @param[in]     ch   Hardware channel.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_IO.
 */
tp_status_t tp_encoder_init(tp_encoder_t *enc, tp_encoder_ch_t ch);

/**
 * @brief Sample the hardware and refresh position and velocity.
 *
 * Must be called at a fixed rate (the sensor task, 500 Hz) because the velocity
 * estimator divides by elapsed time.
 *
 * @param[in,out] enc       Instance.
 * @param[in]     tick_ms   Current millisecond timestamp.
 * @param[in]     elapsed_us Microseconds since the previous call.  Passing 0
 *                          makes the function use @p tick_ms and assume a 1 ms
 *                          grid, which is what the unit tests do.
 * @return @ref TP_OK, or @ref TP_ERR_NOT_READY when the hardware read failed
 *         (the previous values are kept and the error is counted).
 */
tp_status_t tp_encoder_update(tp_encoder_t *enc, uint32_t tick_ms,
                              uint32_t elapsed_us);

/**
 * @brief Absolute joint angle.
 *
 * @param[in] enc  Instance, may be NULL.
 * @return Position in milliradians, or 0.0f when @p enc is NULL.
 */
float tp_encoder_get_position_mrad(const tp_encoder_t *enc);

/**
 * @brief Filtered joint angular velocity.
 *
 * @param[in] enc  Instance, may be NULL.
 * @return Velocity in milliradians per second, or 0.0f when @p enc is NULL.
 */
float tp_encoder_get_speed_mrad_s(const tp_encoder_t *enc);

/**
 * @brief Unfiltered estimator output, for diagnostics only.
 *
 * Comparing this with @ref tp_encoder_get_speed_mrad_s is how you tell whether
 * a noisy velocity reading comes from the estimator or from the plant.
 *
 * @param[in] enc  Instance.
 * @return Raw estimator velocity in milliradians per second.
 */
float tp_encoder_get_speed_raw_mrad_s(const tp_encoder_t *enc);

/**
 * @brief Counts seen in the most recent sample period.
 *
 * @param[in] enc  Instance.
 * @return Signed count delta.
 */
int32_t tp_encoder_get_delta(const tp_encoder_t *enc);

/**
 * @brief Declare the current position to be the new zero.
 *
 * Called after homing.  The hardware counter is reset too, so the software and
 * hardware origins cannot drift apart across a long run.
 *
 * @param[in,out] enc  Instance.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_encoder_set_zero(tp_encoder_t *enc);

/**
 * @brief Convert a distance or angle into encoder counts.
 *
 * Exposed because the position loop's tolerance and the homing search window
 * are both expressed in counts in some places and in mrad in others; having one
 * conversion function stops two definitions from drifting apart.
 *
 * @param[in] enc    Instance.
 * @param[in] mrad   Angle in milliradians.
 * @return Equivalent count value, truncated towards zero.
 */
int32_t tp_encoder_mrad_to_counts(const tp_encoder_t *enc, float mrad);

/**
 * @brief Whether the channel has been declared faulty.
 *
 * @param[in] enc  Instance.
 * @return Non-zero once more than 10 consecutive reads have failed.
 */
int tp_encoder_is_faulty(const tp_encoder_t *enc);

/**
 * @brief Clear the fault latch and the error counters.
 *
 * @param[in,out] enc  Instance.
 */
void tp_encoder_clear_fault(tp_encoder_t *enc);

/**
 * @brief Number of consecutive failed hardware reads.
 *
 * @param[in] enc  Instance.
 * @return Consecutive failure count.
 */
uint32_t tp_encoder_consecutive_errors(const tp_encoder_t *enc);

#ifdef __cplusplus
}
#endif

#endif /* TP_ENCODER_H */
