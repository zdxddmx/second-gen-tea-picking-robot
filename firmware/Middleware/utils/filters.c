/**
 * @file    filters.c
 * @brief   Implementation of the sensor-conditioning filters.
 * @copyright MIT License, see LICENSE.
 */

#include "filters.h"

#include <string.h>

#include "compiler.h"

/** @brief Largest window any of the sliding filters accepts. */
#define FILTER_MAX_WINDOW TP_FILTER_MEDIAN_MAX

/* =========================================================================
 * Moving average
 * ========================================================================= */

tp_status_t tp_filter_mavg_init(tp_filter_mavg_t *f, uint32_t window)
{
    if (f == NULL) {
        return TP_ERR_PARAM;
    }
    (void)memset(f, 0, sizeof(*f));
    if (window == 0u) {
        window = 1u;
    }
    if (window > FILTER_MAX_WINDOW) {
        window = FILTER_MAX_WINDOW;
    }
    f->window = window;
    return TP_OK;
}

void tp_filter_mavg_reset(tp_filter_mavg_t *f)
{
    uint32_t w;

    if (f == NULL) {
        return;
    }
    w = f->window;
    (void)memset(f, 0, sizeof(*f));
    f->window = (w == 0u) ? 1u : w;
}

float tp_filter_mavg_update(tp_filter_mavg_t *f, float x)
{
    uint32_t oldest_index;

    if (f == NULL) {
        return 0.0f;
    }
    if (!TP_ISFINITE(x)) {
        /* Refuse the sample: a NaN in the running sum would poison the filter
         * permanently and every following output would be NaN. */
        return f->value;
    }
    if (f->window == 0u) {
        f->window = 1u;
    }

    if (f->count < f->window) {
        /* Priming phase: the average is taken over the samples actually seen,
         * so the output starts at the true signal level rather than ramping
         * up from zero. */
        f->buffer[f->index] = x;
        f->sum += x;
        f->count++;
        f->index = (f->index + 1u) % f->window;
        f->value = f->sum / (float)f->count;
    } else {
        oldest_index = f->index;
        f->sum -= f->buffer[oldest_index];
        f->sum += x;
        f->buffer[oldest_index] = x;
        f->index = (f->index + 1u) % f->window;
        f->value = f->sum / (float)f->window;
    }
    return f->value;
}

float tp_filter_mavg_value(const tp_filter_mavg_t *f)
{
    return (f != NULL) ? f->value : 0.0f;
}

/* =========================================================================
 * First-order low pass
 * ========================================================================= */

tp_status_t tp_filter_lpf_init(tp_filter_lpf_t *f, float tau_s, float dt_s)
{
    if (f == NULL) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }
    if (!TP_ISFINITE(tau_s) || (tau_s < 0.0f)) {
        return TP_ERR_RANGE;
    }
    (void)memset(f, 0, sizeof(*f));
    f->tau_s = tau_s;
    f->dt_s = dt_s;
    f->alpha = (tau_s > 0.0f) ? (dt_s / (tau_s + dt_s)) : 1.0f;
    return TP_OK;
}

tp_status_t tp_filter_lpf_set_tau(tp_filter_lpf_t *f, float tau_s)
{
    if ((f == NULL) || !TP_ISFINITE(tau_s) || (tau_s < 0.0f)) {
        return TP_ERR_PARAM;
    }
    f->tau_s = tau_s;
    f->alpha = (tau_s > 0.0f) ? (f->dt_s / (tau_s + f->dt_s)) : 1.0f;
    return TP_OK;
}

void tp_filter_lpf_reset(tp_filter_lpf_t *f)
{
    if (f == NULL) {
        return;
    }
    f->value = 0.0f;
    f->primed = 0;
}

float tp_filter_lpf_update(tp_filter_lpf_t *f, float x)
{
    if (f == NULL) {
        return 0.0f;
    }
    if (!TP_ISFINITE(x)) {
        return f->value;
    }
    if (f->primed == 0) {
        /* Absorb the first sample directly; starting from zero would inject a
         * step of the full signal magnitude into the control loop. */
        f->value = x;
        f->primed = 1;
        return f->value;
    }
    f->value += (x - f->value) * f->alpha;
    return f->value;
}

float tp_filter_lpf_value(const tp_filter_lpf_t *f)
{
    return (f != NULL) ? f->value : 0.0f;
}

/* =========================================================================
 * Median
 * ========================================================================= */

tp_status_t tp_filter_median_init(tp_filter_median_t *f, uint32_t length)
{
    if (f == NULL) {
        return TP_ERR_PARAM;
    }
    (void)memset(f, 0, sizeof(*f));
    if (length == 0u) {
        length = 1u;
    }
    if (length > FILTER_MAX_WINDOW) {
        length = FILTER_MAX_WINDOW;
    }
    if ((length & 1u) == 0u) {
        length++; /* A true median needs an odd window. */
    }
    f->length = length;
    return TP_OK;
}

void tp_filter_median_reset(tp_filter_median_t *f)
{
    uint32_t len;

    if (f == NULL) {
        return;
    }
    len = (f->length == 0u) ? 1u : f->length;
    (void)memset(f, 0, sizeof(*f));
    f->length = len;
}

float tp_filter_median_update(tp_filter_median_t *f, float x)
{
    float    sorted[FILTER_MAX_WINDOW];
    uint32_t i;
    uint32_t n;
    uint32_t j;

    if (f == NULL) {
        return 0.0f;
    }
    if (!TP_ISFINITE(x)) {
        return f->value;
    }
    if (f->length == 0u) {
        f->length = 1u;
    }

    f->window[f->index] = x;
    f->index = (f->index + 1u) % f->length;
    if (f->count < f->length) {
        f->count++;
    }

    /* Copy the valid part of the window and insertion-sort it.  At length 9
     * this is at most 36 comparisons, i.e. a few microseconds on an M4. */
    n = f->count;
    for (i = 0u; i < n; ++i) {
        sorted[i] = f->window[i];
    }
    for (i = 1u; i < n; ++i) {
        float key = sorted[i];
        j = i;
        while ((j > 0u) && (sorted[j - 1u] > key)) {
            sorted[j] = sorted[j - 1u];
            j--;
        }
        sorted[j] = key;
    }
    f->value = sorted[n / 2u];
    return f->value;
}

/* =========================================================================
 * Slew-rate limiter
 * ========================================================================= */

tp_status_t tp_filter_slew_init(tp_filter_slew_t *s, float rate_per_s, float dt_s)
{
    if (s == NULL) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(rate_per_s) || (rate_per_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }
    (void)memset(s, 0, sizeof(*s));
    s->rate_per_s = rate_per_s;
    s->dt_s = dt_s;
    return TP_OK;
}

void tp_filter_slew_reset(tp_filter_slew_t *s)
{
    if (s == NULL) {
        return;
    }
    s->value = 0.0f;
    s->primed = 0;
}

float tp_filter_slew_update(tp_filter_slew_t *s, float x)
{
    float max_step;

    if (s == NULL) {
        return 0.0f;
    }
    if (!TP_ISFINITE(x)) {
        return s->value;
    }
    if (s->primed == 0) {
        s->value = x;
        s->primed = 1;
        return s->value;
    }
    max_step = s->rate_per_s * s->dt_s;
    if ((x - s->value) > max_step) {
        s->value += max_step;
    } else if ((s->value - x) > max_step) {
        s->value -= max_step;
    } else {
        s->value = x;
    }
    return s->value;
}

/* =========================================================================
 * Spike detector
 * ========================================================================= */

tp_status_t tp_spike_detector_init(tp_spike_detector_t *d, float threshold)
{
    if (d == NULL) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(threshold) || (threshold <= 0.0f)) {
        return TP_ERR_RANGE;
    }
    (void)memset(d, 0, sizeof(*d));
    d->threshold = threshold;
    return TP_OK;
}

void tp_spike_detector_reset(tp_spike_detector_t *d)
{
    float threshold;

    if (d == NULL) {
        return;
    }
    threshold = (d->threshold > 0.0f) ? d->threshold : 1.0f;
    (void)memset(d, 0, sizeof(*d));
    d->threshold = threshold;
}

int tp_spike_detector_update(tp_spike_detector_t *d, float x)
{
    int is_spike = 0;

    if (d == NULL) {
        return 0;
    }
    if (!TP_ISFINITE(x)) {
        d->spikes++;
        d->streak++;
        return 1;
    }
    if (d->primed == 0) {
        d->prev = x;
        d->primed = 1;
        return 0;
    }
    if (TP_FABS(x - d->prev) > d->threshold) {
        is_spike = 1;
        d->spikes++;
        d->streak++;
    } else {
        d->streak = 0u;
    }
    /*
     * The reference is updated on *every* sample, including a spike.
     *
     * The first revision deliberately did not do this, on the theory that
     * keeping the last plausible value would turn a persistent offset into a
     * spike on every sample and so be detected.  That reasoning is wrong in
     * practice, and the simulation demonstrated it: a single *legitimate* fast
     * transient - the impact when the jaws meet a stem at 40 mm/s - left the
     * reference stranded at the pre-impact value, so every following sample
     * looked like another spike, the streak counter reached eight within 16 ms,
     * and the firmware latched ERR_SENSOR on a perfectly healthy load cell.
     *
     * A persistent offset is not a spike, and detecting it is not this
     * detector's job: a stuck or railed channel is caught by the range check and
     * the saturation flag in `force_sensor.c`, which is where that belongs.
     * What this detector is for is an isolated EMI glitch, and for that the
     * correct rule is simply "a step larger than any physical contact could
     * produce".  With the reference always current, a streak of eight really
     * does mean eight consecutive implausible steps, i.e. a slew rate no
     * mechanical contact can generate.
     */
    d->prev = x;
    return is_spike;
}
