/**
 * @file    test_filters.c
 * @brief   Unit tests for the sensor-conditioning filters and the ring buffer.
 * @copyright MIT License, see LICENSE.
 */

#include "test_framework.h"

#include <math.h>

#include "filters.h"
#include "ring_buffer.h"

/** @brief Sample time used by the filter tests. */
#define DT 0.002f

/* =========================================================================
 * Moving average
 * ========================================================================= */

TP_TEST_CASE(filters_mavg, primes_at_the_first_sample)
{
    tp_filter_mavg_t f;

    TP_ASSERT_EQ(tp_filter_mavg_init(&f, 8u), TP_OK);
    /* The first output must be the first input, not input/window: ramping up
     * from zero looks like a slow contact event to the gripper controller. */
    TP_ASSERT_NEAR(tp_filter_mavg_update(&f, 5.0f), 5.0f, 1.0e-6f);
    TP_ASSERT_NEAR(tp_filter_mavg_update(&f, 5.0f), 5.0f, 1.0e-6f);
}

TP_TEST_CASE(filters_mavg, averages_over_the_window)
{
    tp_filter_mavg_t f;
    int i;
    float v = 0.0f;

    TP_ASSERT_EQ(tp_filter_mavg_init(&f, 4u), TP_OK);
    for (i = 0; i < 4; ++i) {
        v = tp_filter_mavg_update(&f, (float)(i + 1)); /* 1,2,3,4 */
    }
    TP_ASSERT_NEAR(v, 2.5f, 1.0e-6f);
    /* The window is full, so the next sample evicts the oldest. */
    v = tp_filter_mavg_update(&f, 5.0f); /* 2,3,4,5 */
    TP_ASSERT_NEAR(v, 3.5f, 1.0e-6f);
}

TP_TEST_CASE(filters_mavg, window_is_clamped_and_one_is_supported)
{
    tp_filter_mavg_t f;

    TP_ASSERT_EQ(tp_filter_mavg_init(&f, 0u), TP_OK);
    TP_ASSERT_EQ(f.window, 1u);
    TP_ASSERT_NEAR(tp_filter_mavg_update(&f, 3.0f), 3.0f, 1.0e-6f);

    TP_ASSERT_EQ(tp_filter_mavg_init(&f, 1000u), TP_OK);
    TP_ASSERT_EQ(f.window, TP_FILTER_MEDIAN_MAX);

    TP_ASSERT_EQ(tp_filter_mavg_init(NULL, 4u), TP_ERR_PARAM);
}

TP_TEST_CASE(filters_mavg, refuses_non_finite_samples)
{
    tp_filter_mavg_t f;
    float nan_value = 0.0f / 0.0f;

    TP_ASSERT_EQ(tp_filter_mavg_init(&f, 4u), TP_OK);
    (void)tp_filter_mavg_update(&f, 2.0f);
    /* A NaN admitted into the running sum poisons the filter permanently. */
    TP_ASSERT_NEAR(tp_filter_mavg_update(&f, nan_value), 2.0f, 1.0e-6f);
    TP_ASSERT_NEAR(tp_filter_mavg_update(&f, 4.0f), 3.0f, 1.0e-6f);
}

TP_TEST_CASE(filters_mavg, reset_clears_priming)
{
    tp_filter_mavg_t f;

    TP_ASSERT_EQ(tp_filter_mavg_init(&f, 4u), TP_OK);
    (void)tp_filter_mavg_update(&f, 100.0f);
    tp_filter_mavg_reset(&f);
    TP_ASSERT_NEAR(tp_filter_mavg_update(&f, 1.0f), 1.0f, 1.0e-6f);
}

/* =========================================================================
 * First-order low pass
 * ========================================================================= */

TP_TEST_CASE(filters_lpf, alpha_comes_from_the_time_constant)
{
    tp_filter_lpf_t f;

    TP_ASSERT_EQ(tp_filter_lpf_init(&f, 0.006f, 0.002f), TP_OK);
    TP_ASSERT_NEAR(f.alpha, 0.002f / 0.008f, 1.0e-6f);

    /* tau = 0 means "pass through", which is how a caller disables filtering
     * without a separate code path. */
    TP_ASSERT_EQ(tp_filter_lpf_init(&f, 0.0f, 0.002f), TP_OK);
    TP_ASSERT_NEAR(tp_filter_lpf_update(&f, 1.0f), 1.0f, 1.0e-6f);
    TP_ASSERT_NEAR(tp_filter_lpf_update(&f, 9.0f), 9.0f, 1.0e-6f);
}

TP_TEST_CASE(filters_lpf, step_response_follows_the_analytic_curve)
{
    tp_filter_lpf_t f;
    int k;

    TP_ASSERT_EQ(tp_filter_lpf_init(&f, 0.01f, DT), TP_OK);
    (void)tp_filter_lpf_update(&f, 0.0f); /* prime at zero */
    for (k = 1; k <= 50; ++k) {
        float y = tp_filter_lpf_update(&f, 1.0f);
        float t = (float)k * DT;
        float expected = 1.0f - (float)exp(-(double)(t / 0.01f));
        /* Backward Euler *lags* the continuous response by roughly half a
         * sample time, which at tau = 10 ms and dt = 2 ms is a difference of
         * about 1.8 % — largest at the start of the step and decaying from
         * there.  A 1 % band would fail on the first samples and a wider band
         * would stop testing anything, so the tolerance is 4 % and the point of
         * the test is that the *shape* and the time constant are right. */
        TP_ASSERT_NEAR(y, expected, 0.04f);
    }
}

TP_TEST_CASE(filters_lpf, stays_stable_when_dt_exceeds_tau)
{
    tp_filter_lpf_t f;
    int k;

    /* dt = 100x tau is the case where alpha = dt/tau would exceed 1 and the
     * filter would diverge.  The backward-Euler form must not. */
    TP_ASSERT_EQ(tp_filter_lpf_init(&f, 0.0001f, 0.01f), TP_OK);
    (void)tp_filter_lpf_update(&f, 0.0f);
    for (k = 0; k < 100; ++k) {
        float y = tp_filter_lpf_update(&f, 1.0f);
        TP_ASSERT(y >= 0.0f);
        TP_ASSERT(y <= 1.0f);
    }
}

TP_TEST_CASE(filters_lpf, tau_can_be_changed_at_runtime)
{
    tp_filter_lpf_t f;

    TP_ASSERT_EQ(tp_filter_lpf_init(&f, 0.01f, DT), TP_OK);
    TP_ASSERT_EQ(tp_filter_lpf_set_tau(&f, 0.05f), TP_OK);
    TP_ASSERT_NEAR(f.alpha, DT / (0.05f + DT), 1.0e-6f);
    TP_ASSERT_EQ(tp_filter_lpf_set_tau(&f, -1.0f), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_filter_lpf_init(&f, 0.01f, 0.0f), TP_ERR_RANGE);
}

/* =========================================================================
 * Median
 * ========================================================================= */

TP_TEST_CASE(filters_median, removes_a_single_outlier)
{
    tp_filter_median_t f;
    float v = 0.0f;
    int i;

    TP_ASSERT_EQ(tp_filter_median_init(&f, 5u), TP_OK);
    for (i = 0; i < 5; ++i) {
        v = tp_filter_median_update(&f, 1.0f);
    }
    TP_ASSERT_NEAR(v, 1.0f, 1.0e-6f);

    /* A single huge spike must not move the output at all —which is precisely
     * what the moving average *cannot* do. */
    v = tp_filter_median_update(&f, 1000.0f);
    TP_ASSERT_NEAR(v, 1.0f, 1.0e-6f);
    v = tp_filter_median_update(&f, 1.0f);
    TP_ASSERT_NEAR(v, 1.0f, 1.0e-6f);
}

TP_TEST_CASE(filters_median, even_lengths_are_rounded_up)
{
    tp_filter_median_t f;

    TP_ASSERT_EQ(tp_filter_median_init(&f, 4u), TP_OK);
    TP_ASSERT_EQ(f.length, 5u); /* a true median needs an odd window */
    TP_ASSERT_EQ(tp_filter_median_init(&f, 0u), TP_OK);
    TP_ASSERT_EQ(f.length, 1u);
    TP_ASSERT_EQ(tp_filter_median_init(&f, 999u), TP_OK);
    TP_ASSERT_EQ(f.length, TP_FILTER_MEDIAN_MAX);
}

TP_TEST_CASE(filters_median, sorts_correctly_for_an_unsorted_window)
{
    tp_filter_median_t f;
    const float samples[5] = { 5.0f, 1.0f, 4.0f, 2.0f, 3.0f };
    float v = 0.0f;
    int i;

    TP_ASSERT_EQ(tp_filter_median_init(&f, 5u), TP_OK);
    for (i = 0; i < 5; ++i) {
        v = tp_filter_median_update(&f, samples[i]);
    }
    TP_ASSERT_NEAR(v, 3.0f, 1.0e-6f);
}

/* =========================================================================
 * Slew limiter and spike detector
 * ========================================================================= */

TP_TEST_CASE(filters_slew, limits_the_rate_of_change)
{
    tp_filter_slew_t s;
    float v;

    TP_ASSERT_EQ(tp_filter_slew_init(&s, 100.0f, 0.001f), TP_OK);
    /* First sample passes through, then the rate is limited to 100/s. */
    TP_ASSERT_NEAR(tp_filter_slew_update(&s, 0.0f), 0.0f, 1.0e-6f);
    v = tp_filter_slew_update(&s, 1000.0f);
    TP_ASSERT_NEAR(v, 0.1f, 1.0e-6f);
    v = tp_filter_slew_update(&s, 1000.0f);
    TP_ASSERT_NEAR(v, 0.2f, 1.0e-6f);

    TP_ASSERT_EQ(tp_filter_slew_init(&s, 0.0f, 0.001f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_filter_slew_init(&s, 100.0f, 0.0f), TP_ERR_RANGE);
}

TP_TEST_CASE(filters_spike, flags_a_large_step_then_recovers)
{
    tp_spike_detector_t d;

    TP_ASSERT_EQ(tp_spike_detector_init(&d, 3.0f), TP_OK);
    TP_ASSERT_EQ(tp_spike_detector_update(&d, 0.0f), 0);
    TP_ASSERT_EQ(tp_spike_detector_update(&d, 1.0f), 0);
    TP_ASSERT_EQ(tp_spike_detector_update(&d, 100.0f), 1);
    /* The reference must be updated even on a spike.  Keeping it stranded is
     * what made the first revision latch a sensor fault on a legitimate impact:
     * every following sample then looked like another spike. */
    TP_ASSERT_NEAR(d.prev, 100.0f, 1.0e-6f);
    TP_ASSERT_EQ(tp_spike_detector_update(&d, 101.0f), 0);
    TP_ASSERT_EQ(d.streak, 0u);
    TP_ASSERT_EQ(tp_spike_detector_update(&d, 200.0f), 1);
    TP_ASSERT_EQ(d.streak, 1u);
}

TP_TEST_CASE(filters_spike, rejects_bad_configuration)
{
    tp_spike_detector_t d;

    TP_ASSERT_EQ(tp_spike_detector_init(&d, 0.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_spike_detector_init(&d, -1.0f), TP_ERR_RANGE);
    TP_ASSERT_EQ(tp_spike_detector_init(NULL, 1.0f), TP_ERR_PARAM);
}

/* =========================================================================
 * Ring buffer
 * ========================================================================= */

TP_TEST_CASE(ringbuf, push_and_pop_in_order)
{
    tp_ring_buffer_t rb;
    uint8_t storage[16];
    uint8_t out = 0u;
    int i;

    TP_ASSERT_EQ(tp_ring_buffer_init(&rb, storage, sizeof(storage)), TP_OK);
    TP_ASSERT_EQ(rb.capacity, 16u);
    for (i = 0; i < 10; ++i) {
        TP_ASSERT_EQ(tp_ring_buffer_push(&rb, (uint8_t)i), TP_OK);
    }
    TP_ASSERT_EQ(tp_ring_buffer_count(&rb), 10u);
    for (i = 0; i < 10; ++i) {
        TP_ASSERT_EQ(tp_ring_buffer_pop(&rb, &out), TP_OK);
        TP_ASSERT_EQ(out, (uint8_t)i);
    }
    TP_ASSERT_EQ(tp_ring_buffer_pop(&rb, &out), TP_ERR_NOT_READY);
}

TP_TEST_CASE(ringbuf, capacity_is_rounded_down_to_a_power_of_two)
{
    tp_ring_buffer_t rb;
    uint8_t storage[100];

    TP_ASSERT_EQ(tp_ring_buffer_init(&rb, storage, 100u), TP_OK);
    TP_ASSERT_EQ(rb.capacity, 64u);
    TP_ASSERT_EQ(rb.mask, 63u);

    /* A capacity below 2 cannot hold anything meaningful. */
    TP_ASSERT_EQ(tp_ring_buffer_init(&rb, storage, 1u), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_ring_buffer_init(NULL, storage, 16u), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_ring_buffer_init(&rb, NULL, 16u), TP_ERR_PARAM);
}

TP_TEST_CASE(ringbuf, full_buffer_drops_the_new_byte_not_the_old)
{
    tp_ring_buffer_t rb;
    uint8_t storage[4];
    uint8_t out = 0u;
    int i;

    TP_ASSERT_EQ(tp_ring_buffer_init(&rb, storage, sizeof(storage)), TP_OK);
    for (i = 0; i < 4; ++i) {
        TP_ASSERT_EQ(tp_ring_buffer_push(&rb, (uint8_t)(0x10 + i)), TP_OK);
    }
    TP_ASSERT(tp_ring_buffer_is_full(&rb) != 0);
    /* The docstring promises the *new* byte is dropped: overwriting old data
     * would corrupt a frame that was already partially accepted. */
    TP_ASSERT_EQ(tp_ring_buffer_push(&rb, 0xFFu), TP_ERR_OVERFLOW);
    TP_ASSERT_EQ(tp_ring_buffer_overflow_count(&rb), 1u);
    TP_ASSERT_EQ(tp_ring_buffer_pop(&rb, &out), TP_OK);
    TP_ASSERT_EQ(out, 0x10u);
}

TP_TEST_CASE(ringbuf, block_write_is_all_or_nothing)
{
    tp_ring_buffer_t rb;
    uint8_t storage[8];
    const uint8_t block[6] = { 1, 2, 3, 4, 5, 6 };

    TP_ASSERT_EQ(tp_ring_buffer_init(&rb, storage, sizeof(storage)), TP_OK);
    TP_ASSERT_EQ(tp_ring_buffer_write(&rb, block, 6u), TP_OK);
    TP_ASSERT_EQ(tp_ring_buffer_count(&rb), 6u);

    /* A 6-byte block cannot fit in 2 bytes of space; nothing may be written. */
    TP_ASSERT_EQ(tp_ring_buffer_write(&rb, block, 6u), TP_ERR_OVERFLOW);
    TP_ASSERT_EQ(tp_ring_buffer_count(&rb), 6u);
    TP_ASSERT_EQ(tp_ring_buffer_overflow_count(&rb), 6u);
}

TP_TEST_CASE(ringbuf, peek_does_not_consume)
{
    tp_ring_buffer_t rb;
    uint8_t storage[8];
    uint8_t out = 0u;

    TP_ASSERT_EQ(tp_ring_buffer_init(&rb, storage, sizeof(storage)), TP_OK);
    (void)tp_ring_buffer_push(&rb, 0xAAu);
    (void)tp_ring_buffer_push(&rb, 0xBBu);

    TP_ASSERT_EQ(tp_ring_buffer_peek(&rb, 0u, &out), TP_OK);
    TP_ASSERT_EQ(out, 0xAAu);
    TP_ASSERT_EQ(tp_ring_buffer_peek(&rb, 1u, &out), TP_OK);
    TP_ASSERT_EQ(out, 0xBBu);
    TP_ASSERT_EQ(tp_ring_buffer_peek(&rb, 2u, &out), TP_ERR_NOT_READY);
    TP_ASSERT_EQ(tp_ring_buffer_count(&rb), 2u);
}

TP_TEST_CASE(ringbuf, high_water_mark_is_tracked)
{
    tp_ring_buffer_t rb;
    uint8_t storage[16];
    int i;

    TP_ASSERT_EQ(tp_ring_buffer_init(&rb, storage, sizeof(storage)), TP_OK);
    for (i = 0; i < 12; ++i) {
        (void)tp_ring_buffer_push(&rb, (uint8_t)i);
    }
    (void)tp_ring_buffer_discard(&rb, 10u);
    for (i = 0; i < 3; ++i) {
        (void)tp_ring_buffer_push(&rb, (uint8_t)i);
    }
    /* Peak occupancy must be remembered even after draining: it is the evidence
     * used to size the buffer from data rather than by guessing. */
    TP_ASSERT_EQ(tp_ring_buffer_high_water(&rb), 12u);
}

TP_TEST_CASE(ringbuf, wraps_many_times_without_corruption)
{
    tp_ring_buffer_t rb;
    uint8_t storage[8];
    uint8_t out = 0u;
    int i;

    TP_ASSERT_EQ(tp_ring_buffer_init(&rb, storage, sizeof(storage)), TP_OK);
    for (i = 0; i < 500; ++i) {
        TP_ASSERT_EQ(tp_ring_buffer_push(&rb, (uint8_t)(i & 0xFF)), TP_OK);
        TP_ASSERT_EQ(tp_ring_buffer_pop(&rb, &out), TP_OK);
        TP_ASSERT_EQ(out, (uint8_t)(i & 0xFF));
    }
    TP_ASSERT(tp_ring_buffer_is_empty(&rb) != 0);
    /* The monotonic head/tail counters must not have wrapped into a false
     * "full" or "empty" reading after hundreds of cycles. */
    TP_ASSERT_EQ(tp_ring_buffer_overflow_count(&rb), 0u);
}
