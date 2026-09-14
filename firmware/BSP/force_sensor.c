/**
 * @file    force_sensor.c
 * @brief   Implementation of the load-cell force channel.
 * @copyright MIT License, see LICENSE.
 */

#include "force_sensor.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

/** @brief Design scale, used to sanity-check a calibration attempt. */
#define FORCE_SCALE_DESIGN TP_FORCE_CAL_SCALE_N_PER_COUNT

/** @brief A calibration is rejected if it is off by more than this factor. */
#define FORCE_SCALE_PLAUSIBLE_FACTOR 10.0f

/**
 * @brief Spike threshold in newtons: no real contact can step faster.
 *
 * At the 500 Hz sensor rate this is a slew-rate limit of 3.0 N * 500 = 1500 N/s.
 * For comparison, the hardest contact this machine can produce is the jaws
 * meeting a stem at the 40 mm/s fast-approach speed against a 0.30 N/mm stem,
 * which is 12 N/s — two orders of magnitude below the threshold.  A glitch from
 * a PWM-coupled EMI burst, by contrast, steps the reading by volts, which at
 * 100.5 mV/N is tens of newtons.
 *
 * The first revision used 1.0 N, which is only 500 N/s and is *close enough to
 * the real impact transient to false-trip*.  It did, in simulation, at the
 * moment of contact.
 */
#define FORCE_SPIKE_THRESHOLD_N 3.0f

/** @brief Consecutive spikes before the channel is declared faulty. */
#define FORCE_SPIKE_TRIP 8u

tp_status_t tp_force_sensor_init(tp_force_sensor_t *fs, float dt_s)
{
    tp_status_t st;

    if (fs == NULL) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }

    (void)memset(fs, 0, sizeof(*fs));
    fs->adc_zero_counts = TP_FORCE_CAL_ZERO_COUNTS;
    fs->n_per_count = TP_FORCE_CAL_SCALE_N_PER_COUNT;
    fs->calibrated = 0; /* design values, not a real calibration */
    fs->min_n = 0.0f;
    fs->max_n = 0.0f;

    st = tp_filter_median_init(&fs->median, TP_FILTER_MEDIAN_WINDOW);
    if (st != TP_OK) {
        return st;
    }
    st = tp_filter_mavg_init(&fs->mavg, TP_FILTER_FORCE_MA_WINDOW);
    if (st != TP_OK) {
        return st;
    }
    st = tp_filter_lpf_init(&fs->lpf, TP_FILTER_FORCE_LPF_TAU_S, dt_s);
    if (st != TP_OK) {
        return st;
    }
    st = tp_spike_detector_init(&fs->spikes, FORCE_SPIKE_THRESHOLD_N);
    if (st != TP_OK) {
        return st;
    }

    fs->initialised = 1;
    LOG_INFO(TP_MOD_FORCE,
             "channel ready: zero %.1f counts, %.5f N/count (%.2f mN LSB), range +-%.1f N",
             (double)fs->adc_zero_counts, (double)fs->n_per_count,
             (double)(fs->n_per_count * 1000.0f), (double)TP_FORCE_USABLE_RANGE_N);
    return TP_OK;
}

float tp_force_sensor_counts_to_n(const tp_force_sensor_t *fs, uint16_t counts)
{
    if (fs == NULL) {
        return 0.0f;
    }
    return ((float)counts - fs->adc_zero_counts) * fs->n_per_count;
}

tp_status_t tp_force_sensor_update(tp_force_sensor_t *fs, uint16_t counts)
{
    float raw_n;
    float value;
    int   spike;

    if ((fs == NULL) || (fs->initialised == 0)) {
        return TP_ERR_NOT_READY;
    }

    fs->adc_counts = counts;
    fs->sample_count++;

    /* --- 1. plausibility and saturation -------------------------------- */
    fs->saturated = ((counts == 0u) || (counts >= (uint16_t)TP_ADC_FULL_SCALE))
                  ? 1 : 0;
    if (fs->saturated != 0) {
        /*
         * A conversion pinned at either rail is not a force measurement, it is a
         * disconnected bridge, a shorted amplifier input or a dead reference.
         *
         * This has to be a fault in its own right, and the reason is a gap the
         * unit tests found: the ±20 N plausibility window is *wider* than the
         * value a rail produces.  At 8.02 mN per count, mid-scale minus 2048
         * counts is -16.4 N, which sits comfortably inside the window — so a
         * wire broken to ground presented itself as a legitimate large negative
         * force, and the force controller would have believed it.
         */
        if (fs->fault == 0) {
            LOG_ERROR(TP_MOD_FORCE,
                      "channel declared faulty: ADC pinned at %u counts (rail)",
                      (unsigned)counts);
        }
        fs->fault = 1;
        fs->fault_count++;
    }

    /* --- 2. spike detection on the RAW value --------------------------- */
    raw_n = tp_force_sensor_counts_to_n(fs, counts);
    spike = tp_spike_detector_update(&fs->spikes, raw_n);
    if (spike != 0) {
        if (fs->spikes.streak >= FORCE_SPIKE_TRIP) {
            if (fs->fault == 0) {
                LOG_ERROR(TP_MOD_FORCE,
                          "channel declared faulty: %lu consecutive spikes > %.2f N",
                          (unsigned long)fs->spikes.streak,
                          (double)FORCE_SPIKE_THRESHOLD_N);
            }
            fs->fault = 1;
            fs->fault_count++;
        }
        /* The sample is still fed to the filter chain: the median stage exists
         * precisely to remove it, and skipping it here would leave a gap that
         * the moving average would fill with stale data. */
    }

    /* --- 3. range check ------------------------------------------------ */
    if ((raw_n < TP_SENSOR_FORCE_MIN_N) || (raw_n > TP_SENSOR_FORCE_MAX_N)) {
        if (fs->fault == 0) {
            LOG_ERROR(TP_MOD_FORCE, "reading %.3f N outside plausible [%.1f, %.1f] N",
                      (double)raw_n, (double)TP_SENSOR_FORCE_MIN_N,
                      (double)TP_SENSOR_FORCE_MAX_N);
        }
        fs->fault = 1;
        fs->fault_count++;
    }

    if (!TP_ISFINITE(raw_n)) {
        fs->rejected_count++;
        return TP_ERR_NOT_READY;
    }
    fs->force_raw_n = raw_n;

    /* --- 4. filter chain ---------------------------------------------- */
    value = tp_filter_median_update(&fs->median, raw_n);
    value = tp_filter_mavg_update(&fs->mavg, value);
    value = tp_filter_lpf_update(&fs->lpf, value);
    if (!TP_ISFINITE(value)) {
        fs->rejected_count++;
        return TP_ERR_NOT_READY;
    }
    fs->force_n = value;

    /* --- 5. statistics ------------------------------------------------- */
    if (fs->sample_count == 1u) {
        fs->min_n = value;
        fs->max_n = value;
    } else {
        if (value < fs->min_n) {
            fs->min_n = value;
        }
        if (value > fs->max_n) {
            fs->max_n = value;
        }
    }
    return TP_OK;
}

float tp_force_sensor_get_force_n(const tp_force_sensor_t *fs)
{
    return (fs != NULL) ? fs->force_n : 0.0f;
}

float tp_force_sensor_get_force_raw_n(const tp_force_sensor_t *fs)
{
    return (fs != NULL) ? fs->force_raw_n : 0.0f;
}

uint16_t tp_force_sensor_get_counts(const tp_force_sensor_t *fs)
{
    return (fs != NULL) ? fs->adc_counts : 0u;
}

tp_status_t tp_force_sensor_zero_calibrate(tp_force_sensor_t *fs)
{
    uint32_t i;
    uint32_t sum = 0u;
    uint16_t min_counts = 0xFFFFu;
    uint16_t max_counts = 0u;
    float    mean_counts;
    float    spread_n;

    if ((fs == NULL) || (fs->initialised == 0)) {
        return TP_ERR_PARAM;
    }

    for (i = 0u; i < TP_FORCE_ZERO_CAL_SAMPLES; ++i) {
        uint16_t counts = 0u;
        if (tp_platform_adc_read(TP_ADC_FORCE, &counts) != TP_OK) {
            LOG_ERROR(TP_MOD_FORCE, "zero calibration aborted: ADC read failed");
            return TP_ERR_IO;
        }
        sum += counts;
        if (counts < min_counts) {
            min_counts = counts;
        }
        if (counts > max_counts) {
            max_counts = counts;
        }
    }

    mean_counts = (float)sum / (float)TP_FORCE_ZERO_CAL_SAMPLES;
    spread_n = (float)(max_counts - min_counts) * fs->n_per_count;
    fs->last_zero_spread_n = spread_n;

    if (spread_n > TP_FORCE_ZERO_CAL_MAX_SPREAD_N) {
        /* Refusing here is the whole point of the check: averaging a signal
         * that is still moving (jaws closing, a bud already held, someone
         * leaning on the bench) produces an offset that is wrong by the
         * motion's mean, and every later force reading inherits the error. */
        LOG_WARN(TP_MOD_FORCE,
                 "zero calibration rejected: spread %.3f N > %.3f N (signal moving?)",
                 (double)spread_n, (double)TP_FORCE_ZERO_CAL_MAX_SPREAD_N);
        return TP_ERR_STATE;
    }
    if ((mean_counts <= (float)TP_SENSOR_ADC_MIN_COUNTS) ||
        (mean_counts >= (float)TP_SENSOR_ADC_MAX_COUNTS)) {
        LOG_ERROR(TP_MOD_FORCE,
                  "zero calibration rejected: mean %.1f counts is at a rail", 
                  (double)mean_counts);
        return TP_ERR_STATE;
    }

    fs->adc_zero_counts = mean_counts;
    fs->calibrated = 1;
    fs->last_zero_cal_ms = tp_platform_time_ms();
    fs->fault = 0;
    tp_force_sensor_reset_tracking(fs);
    LOG_INFO(TP_MOD_FORCE,
             "zero calibrated: %.1f counts from %lu samples, spread %.3f N",
             (double)mean_counts, (unsigned long)TP_FORCE_ZERO_CAL_SAMPLES,
             (double)spread_n);
    return TP_OK;
}

tp_status_t tp_force_sensor_calibrate_scale(tp_force_sensor_t *fs,
                                            float known_force_n)
{
    float counts;
    float delta_counts;
    float new_scale;
    uint16_t adc = 0u;

    if ((fs == NULL) || (fs->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(known_force_n) || (TP_FABS(known_force_n) < 1.0e-3f)) {
        return TP_ERR_RANGE;
    }
    if (tp_platform_adc_read(TP_ADC_FORCE, &adc) != TP_OK) {
        return TP_ERR_IO;
    }

    counts = (float)adc;
    delta_counts = counts - fs->adc_zero_counts;
    if (TP_FABS(delta_counts) < 1.0f) {
        /* Fewer than one count of deflection means the applied load is not
         * reaching the sensor; solving for a scale here would produce an
         * enormous, meaningless number. */
        LOG_WARN(TP_MOD_FORCE, "scale calibration rejected: deflection %.1f counts",
                 (double)delta_counts);
        return TP_ERR_STATE;
    }

    new_scale = known_force_n / delta_counts;
    if ((TP_FABS(new_scale) > (FORCE_SCALE_PLAUSIBLE_FACTOR * FORCE_SCALE_DESIGN)) ||
        (TP_FABS(new_scale) < (FORCE_SCALE_DESIGN / FORCE_SCALE_PLAUSIBLE_FACTOR))) {
        LOG_WARN(TP_MOD_FORCE,
                 "scale calibration rejected: %.6f N/count is %.1fx the design value",
                 (double)new_scale,
                 (double)(new_scale / FORCE_SCALE_DESIGN));
        return TP_ERR_RANGE;
    }

    fs->n_per_count = new_scale;
    fs->calibrated = 1;
    LOG_INFO(TP_MOD_FORCE,
             "scale calibrated: %.5f N/count from %.3f N over %.1f counts",
             (double)new_scale, (double)known_force_n, (double)delta_counts);
    return TP_OK;
}

tp_status_t tp_force_sensor_set_scale(tp_force_sensor_t *fs, float n_per_count)
{
    if ((fs == NULL) || (fs->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(n_per_count) || (n_per_count <= 0.0f)) {
        return TP_ERR_RANGE;
    }
    if ((n_per_count > (FORCE_SCALE_PLAUSIBLE_FACTOR * FORCE_SCALE_DESIGN)) ||
        (n_per_count < (FORCE_SCALE_DESIGN / FORCE_SCALE_PLAUSIBLE_FACTOR))) {
        LOG_WARN(TP_MOD_FORCE, "scale %.6f rejected as implausible (design %.6f)",
                 (double)n_per_count, (double)FORCE_SCALE_DESIGN);
        return TP_ERR_RANGE;
    }
    fs->n_per_count = n_per_count;
    return TP_OK;
}

tp_status_t tp_force_sensor_set_zero(tp_force_sensor_t *fs, float zero_counts)
{
    if ((fs == NULL) || (fs->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(zero_counts) || (zero_counts < 0.0f) ||
        (zero_counts > TP_ADC_FULL_SCALE)) {
        return TP_ERR_RANGE;
    }
    fs->adc_zero_counts = zero_counts;
    return TP_OK;
}

void tp_force_sensor_reset_tracking(tp_force_sensor_t *fs)
{
    if (fs == NULL) {
        return;
    }
    tp_filter_median_reset(&fs->median);
    tp_filter_mavg_reset(&fs->mavg);
    tp_filter_lpf_reset(&fs->lpf);
    tp_spike_detector_reset(&fs->spikes);
    fs->force_n = 0.0f;
    fs->force_raw_n = 0.0f;
    fs->min_n = 0.0f;
    fs->max_n = 0.0f;
    fs->sample_count = 0u;
}

int tp_force_sensor_is_faulty(const tp_force_sensor_t *fs)
{
    return (fs != NULL) ? fs->fault : 1;
}

tp_status_t tp_force_sensor_clear_fault(tp_force_sensor_t *fs)
{
    if ((fs == NULL) || (fs->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if ((fs->adc_counts <= (uint16_t)TP_SENSOR_ADC_MIN_COUNTS) ||
        (fs->adc_counts >= (uint16_t)TP_SENSOR_ADC_MAX_COUNTS)) {
        /* Still pinned at a rail: the wire is probably still broken. */
        return TP_ERR_FAULT_ACTIVE;
    }
    fs->fault = 0;
    tp_spike_detector_reset(&fs->spikes);
    return TP_OK;
}

uint32_t tp_force_sensor_spike_count(const tp_force_sensor_t *fs)
{
    return (fs != NULL) ? fs->spikes.spikes : 0u;
}
