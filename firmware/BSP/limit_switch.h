/**
 * @file    limit_switch.h
 * @brief   Debounced discrete-input reader for limit switches and the E-stop.
 *
 * @details
 * # Why debouncing belongs in firmware
 * A mechanical micro-switch is a spring-loaded contact: when it closes, the
 * metal actually bounces for 0.5-5 ms and the input reads a burst of random
 * ones and zeros.  A 1 kHz control loop samples that burst, so a naive read
 * produces dozens of spurious "limit hit" events per real actuation.  The
 * consequences are not cosmetic — the safety layer would raise `ERR_LIMIT`,
 * latch it, and refuse to move.
 *
 * # The algorithm
 * A **saturating up/down counter** per input (the classic "N-of-M majority"
 * filter):
 *
 *   - read the raw pin,
 *   - increment the counter when it matches the last *stable* state, decrement
 *     otherwise (saturating at 0 and at @ref TP_LIMIT_DEBOUNCE_COUNT),
 *   - publish a new stable state when the counter reaches either extreme.
 *
 * This costs one byte and two comparisons per input per call, which is why it
 * is preferred over a shift-register majority voter on a microcontroller with
 * no barrel shifter benefits to exploit here.
 *
 * # Polarity
 * The E-stop is a **normally-closed** loop: a broken wire reads as "pressed".
 * Switches are normally-open to ground with a pull-up, so an unplugged switch
 * reads "not hit" — which is why @ref tp_limit_switch_init takes an explicit
 * active-low flag per input and why the two cases are documented separately
 * rather than hidden behind one boolean.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_LIMIT_SWITCH_H
#define TP_LIMIT_SWITCH_H

#include <stdint.h>

#include "bsp_types.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Consecutive agreeing samples needed to change a stable state. */
#define TP_LIMIT_DEBOUNCE_COUNT 3u

/**
 * @brief Debounced state of one input.
 */
typedef struct {
    tp_io_id_t id;             /**< Input being watched.                    */
    int        active_low;     /**< Non-zero when a low level means "active".*/
    uint8_t    raw;            /**< Latest raw level.                       */
    uint8_t    stable;         /**< Debounced level.                        */
    uint8_t    counter;        /**< Debounce accumulator.                   */
    uint32_t   transitions;    /**< Number of accepted state changes.       */
    uint32_t   rejected_glitches;/**< Raw changes that never became stable.  */
    uint32_t   update_count;   /**< Calls to @ref tp_limit_switch_update.   */
    int        initialised;    /**< Non-zero after a successful init.       */
} tp_limit_switch_t;

/**
 * @brief Initialise and prime a debounced input.
 *
 * The first read is taken as the initial stable state, so the robot does not
 * see a phantom "limit hit" at boot while the counter fills.
 *
 * @param[in,out] sw          Instance.
 * @param[in]     id          Input to watch.
 * @param[in]     active_low  Non-zero when a low level means asserted.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_NOT_READY when the
 *         backend has no such input.
 */
tp_status_t tp_limit_switch_init(tp_limit_switch_t *sw, tp_io_id_t id,
                                 int active_low);

/**
 * @brief Sample and debounce.
 *
 * @param[in,out] sw  Instance.
 * @return @ref TP_OK, or @ref TP_ERR_NOT_READY / @ref TP_ERR_PARAM.
 */
tp_status_t tp_limit_switch_update(tp_limit_switch_t *sw);

/**
 * @brief Debounced state of the input.
 *
 * @param[in] sw  Instance.
 * @return 1 when asserted (limit hit / E-stop pressed), 0 otherwise.
 */
int tp_limit_switch_is_active(const tp_limit_switch_t *sw);

/**
 * @brief Latest raw level, before debouncing, for diagnostics.
 *
 * @param[in] sw  Instance.
 * @return Raw level.
 */
uint8_t tp_limit_switch_raw(const tp_limit_switch_t *sw);

/**
 * @brief Assert the input's *logical* state directly, bypassing the hardware.
 *
 * Provided for unit tests, which must be able to exercise the debounce logic
 * and the safety reaction without a real switch.  It is not used by any
 * production code path.
 *
 * @param[in,out] sw      Instance.
 * @param[in]     active  Logical level to inject.
 */
void tp_limit_switch_force(tp_limit_switch_t *sw, int active);

#ifdef __cplusplus
}
#endif

#endif /* TP_LIMIT_SWITCH_H */
