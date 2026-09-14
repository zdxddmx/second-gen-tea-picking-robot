/**
 * @file    encoder.c
 * @brief   Implementation of the quadrature encoder driver.
 * @copyright MIT License, see LICENSE.
 */

#include "encoder.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

/** @brief Consecutive read failures that trip the fault latch. */
#define ENCODER_ERROR_TRIP 10u

/**
 * @brief Sign of an integer without branching on the common path.
 *
 * @param[in] v  Value.
 * @return -1, 0 or +1.
 */
static int sign_i32(int32_t v)
{
    return (v > 0) ? 1 : ((v < 0) ? -1 : 0);
}

tp_status_t tp_encoder_init(tp_encoder_t *enc, tp_encoder_ch_t ch)
{
    int32_t count = 0;
    tp_status_t st;

    if (enc == NULL) {
        return TP_ERR_PARAM;
    }
    if (((int)ch < 0) || (ch >= TP_ENCODER_COUNT)) {
        return TP_ERR_PARAM;
    }

    (void)memset(enc, 0, sizeof(*enc));
    enc->ch = ch;
    enc->counts_per_rev = TP_ENCODER_COUNTS_PER_REV;
    enc->mrad_per_count = TP_MRAD_PER_COUNT;

    st = tp_platform_encoder_read(ch, &count);
    if (st != TP_OK) {
        LOG_ERROR(TP_MOD_ENCODER, "channel %d read failed at init (%s)",
                  (int)ch, tp_status_str(st));
        enc->fault = 1;
        return st;
    }
    enc->count = count;
    enc->count_prev = count;
    enc->position_mrad = (float)count * enc->mrad_per_count;

    /* Two cascaded filters: the moving average removes the estimator's
     * stair-step, the low-pass removes broadband noise.  See encoder.h for why
     * both are needed and what each one costs in phase lag. */
    st = tp_filter_mavg_init(&enc->speed_mavg, TP_FILTER_SPEED_MA_WINDOW);
    if (st != TP_OK) {
        return st;
    }
    st = tp_filter_lpf_init(&enc->speed_lpf, TP_FILTER_SPEED_LPF_TAU_S,
                           1.0f / (float)TP_SENSOR_FREQ_HZ);
    if (st != TP_OK) {
        return st;
    }
    enc->initialised = 1;
    LOG_INFO(TP_MOD_ENCODER, "channel %d online, %ld counts/rev, %.5f mrad/count",
             (int)ch, (long)enc->counts_per_rev, (double)enc->mrad_per_count);
    return TP_OK;
}

tp_status_t tp_encoder_update(tp_encoder_t *enc, uint32_t tick_ms,
                              uint32_t elapsed_us)
{
    int32_t count = 0;
    uint32_t dt_us;
    tp_status_t st;
    float estimate;

    if ((enc == NULL) || (enc->initialised == 0)) {
        return TP_ERR_NOT_READY;
    }

    st = tp_platform_encoder_read(enc->ch, &count);
    if (st != TP_OK) {
        enc->read_errors++;
        if (enc->read_errors >= ENCODER_ERROR_TRIP) {
            if (enc->fault == 0) {
                LOG_ERROR(TP_MOD_ENCODER, "channel %d declared faulty after %lu errors",
                          (int)enc->ch, (unsigned long)enc->read_errors);
            }
            enc->fault = 1;
        }
        return TP_ERR_NOT_READY;
    }
    enc->read_errors = 0u;

    enc->count_prev = enc->count;
    enc->count = count;
    enc->delta = enc->count - enc->count_prev;
    enc->position_mrad = ((float)enc->count * enc->mrad_per_count)
                       - enc->position_offset_mrad;
    enc->sample_count++;

    /* --- velocity estimator ------------------------------------------- */
    if (elapsed_us > 0u) {
        dt_us = elapsed_us;
    } else {
        /* Fall back to the millisecond timestamps: correct for a 1 kHz grid and
         * good enough for a unit test that steps the driver by hand. */
        dt_us = (tick_ms - enc->last_tick_ms) * 1000u;
        if (dt_us == 0u) {
            dt_us = 1000000u / TP_SENSOR_FREQ_HZ;
        }
    }
    enc->last_tick_ms = tick_ms;
    enc->accum_counts += enc->delta;
    enc->accum_us += dt_us;

    if ((enc->accum_us == 0u) || (dt_us == 0u)) {
        return TP_OK;
    }

    if ((TP_FABS((float)enc->accum_counts) >= (float)TP_ENCODER_VEL_MIN_INC) ||
        ((enc->accum_us / 1000u) >= TP_ENCODER_VEL_MAX_WINDOW_MS)) {
        /* Publish a fresh estimate and restart the accumulation window.
         * mrad/s = counts * (mrad/count) / (us / 1e6) */
        estimate = ((float)enc->accum_counts * enc->mrad_per_count)
                 / ((float)enc->accum_us / 1000000.0f);
        enc->accum_counts = 0;
        enc->accum_us = 0u;

        if (!TP_ISFINITE(estimate)) {
            estimate = enc->velocity_raw_mrad_s; /* refuse the sample */
        } else {
            int dir = sign_i32((int32_t)estimate);
            if ((dir != 0) && (enc->last_direction != 0) &&
                (dir != enc->last_direction)) {
                enc->direction_reversals++;
            }
            if (dir != 0) {
                enc->last_direction = dir;
            }
        }
        enc->velocity_raw_mrad_s = estimate;

        /* Reset the moving average on a direction change so the filter does not
         * average +v with -v and report a spurious near-zero speed in the
         * middle of a reversal — which the stall detector would believe. */
        if ((enc->last_direction != 0) &&
            (sign_i32((int32_t)estimate) != 0) &&
            (sign_i32((int32_t)estimate) !=
             sign_i32((int32_t)tp_filter_mavg_value(&enc->speed_mavg))) &&
            (TP_FABS(tp_filter_mavg_value(&enc->speed_mavg)) > 1.0f)) {
            tp_filter_mavg_reset(&enc->speed_mavg);
        }

        estimate = tp_filter_mavg_update(&enc->speed_mavg, estimate);
        estimate = tp_filter_lpf_update(&enc->speed_lpf, estimate);
        enc->velocity_mrad_s = TP_ISFINITE(estimate) ? estimate
                                                     : enc->velocity_mrad_s;
    }
    /* else: hold the previous estimate (zero-order hold). */

    return TP_OK;
}

float tp_encoder_get_position_mrad(const tp_encoder_t *enc)
{
    return (enc != NULL) ? enc->position_mrad : 0.0f;
}

float tp_encoder_get_speed_mrad_s(const tp_encoder_t *enc)
{
    return (enc != NULL) ? enc->velocity_mrad_s : 0.0f;
}

float tp_encoder_get_speed_raw_mrad_s(const tp_encoder_t *enc)
{
    return (enc != NULL) ? enc->velocity_raw_mrad_s : 0.0f;
}

int32_t tp_encoder_get_delta(const tp_encoder_t *enc)
{
    return (enc != NULL) ? enc->delta : 0;
}

tp_status_t tp_encoder_set_zero(tp_encoder_t *enc)
{
    if ((enc == NULL) || (enc->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    /* Clear the hardware counter first: if that fails the software origin
     * would be relative to a stale hardware zero. */
    if (tp_platform_encoder_reset(enc->ch) != TP_OK) {
        return TP_ERR_IO;
    }
    enc->count = 0;
    enc->count_prev = 0;
    enc->delta = 0;
    enc->accum_counts = 0;
    enc->accum_us = 0u;
    enc->position_offset_mrad = 0.0f;
    enc->position_mrad = 0.0f;
    tp_filter_mavg_reset(&enc->speed_mavg);
    tp_filter_lpf_reset(&enc->speed_lpf);
    enc->velocity_raw_mrad_s = 0.0f;
    enc->velocity_mrad_s = 0.0f;
    LOG_INFO(TP_MOD_ENCODER, "channel %d zeroed", (int)enc->ch);
    return TP_OK;
}

int32_t tp_encoder_mrad_to_counts(const tp_encoder_t *enc, float mrad)
{
    float counts;

    if ((enc == NULL) || !TP_ISFINITE(mrad) || (enc->mrad_per_count <= 0.0f)) {
        return 0;
    }
    counts = mrad / enc->mrad_per_count;
    if (counts > 2147483647.0f) {
        return 2147483647;
    }
    if (counts < -2147483648.0f) {
        return -2147483648;
    }
    return (int32_t)counts;
}

int tp_encoder_is_faulty(const tp_encoder_t *enc)
{
    return (enc != NULL) ? enc->fault : 1;
}

void tp_encoder_clear_fault(tp_encoder_t *enc)
{
    if (enc == NULL) {
        return;
    }
    enc->fault = 0;
    enc->read_errors = 0u;
}

uint32_t tp_encoder_consecutive_errors(const tp_encoder_t *enc)
{
    return (enc != NULL) ? enc->read_errors : 0u;
}
