/**
 * @file    adc_driver.c
 * @brief   Implementation of the ADC acquisition front end.
 * @copyright MIT License, see LICENSE.
 */

#include "adc_driver.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

tp_status_t tp_adc_channel_init(tp_adc_channel_t *adc, tp_adc_ch_t ch, float dt_s,
                                float divider, uint32_t dma_rate_hz)
{
    uint32_t dec;

    if (adc == NULL) {
        return TP_ERR_PARAM;
    }
    if (((int)ch < 0) || (ch >= TP_ADC_COUNT)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f) || (dma_rate_hz == 0u)) {
        return TP_ERR_RANGE;
    }
    if (!TP_ISFINITE(divider) || (divider <= 0.0f)) {
        return TP_ERR_RANGE;
    }

    (void)memset(adc, 0, sizeof(*adc));
    adc->ch = ch;
    adc->vref_v = TP_BSP_ADC_VREF_V;
    adc->full_scale = TP_ADC_FULL_SCALE;
    adc->divider = divider;

    /* Decimation = how many DMA samples fit in one consumer period.  Clamped
     * to at least 1 so a mis-set rate cannot produce a divide-by-zero. */
    dec = (uint32_t)((float)dma_rate_hz * dt_s + 0.5f);
    if (dec == 0u) {
        dec = 1u;
    }
    if (dec > TP_ADC_BLOCK_MAX) {
        dec = TP_ADC_BLOCK_MAX;
    }
    adc->decimation = dec;
    adc->initialised = 1;
    return TP_OK;
}

float tp_adc_channel_counts_to_volts(const tp_adc_channel_t *adc, uint16_t counts)
{
    if (adc == NULL) {
        return 0.0f;
    }
    return ((float)counts / (adc->full_scale + 1.0f)) * adc->vref_v * adc->divider;
}

tp_status_t tp_adc_channel_push(tp_adc_channel_t *adc, uint16_t counts)
{
    uint32_t average;

    if ((adc == NULL) || (adc->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    adc->sample_count++;
    adc->pending_sum += counts;
    adc->pending++;

    if (adc->pending < adc->decimation) {
        return TP_OK; /* still filling the decimation window */
    }

    average = (adc->pending_sum + (adc->pending / 2u)) / adc->pending;
    adc->pending = 0u;
    adc->pending_sum = 0u;
    if (average > (uint32_t)TP_ADC_FULL_SCALE) {
        average = (uint32_t)TP_ADC_FULL_SCALE;
        adc->clamped++;
    }
    adc->raw = (uint16_t)average;
    adc->volts = tp_adc_channel_counts_to_volts(adc, adc->raw);
    return TP_OK;
}

tp_status_t tp_adc_channel_service(tp_adc_channel_t *adc)
{
    uint32_t n;
    uint32_t i;

    if ((adc == NULL) || (adc->initialised == 0)) {
        return TP_ERR_PARAM;
    }

    n = tp_platform_adc_read_block(adc->ch, adc->block, TP_ADC_BLOCK_MAX);
    if (n == 0u) {
        /* No block is ready.  On the target this means the DMA half-transfer
         * interrupt has not fired yet, which is normal at the very start but a
         * real miss thereafter — hence the counter rather than a log line. */
        adc->overruns++;
        return TP_ERR_NOT_READY;
    }
    adc->block_len = n;

    /* If the DMA produced far more samples than one period needs, the consumer
     * was late and the buffer may already have wrapped.  Consume only the most
     * recent `decimation` samples: old data is worse than no data for a
     * feedback loop, and using it would inject a variable transport delay. */
    if (n > adc->decimation) {
        uint32_t start = n - adc->decimation;
        adc->overruns++;
        for (i = start; i < n; ++i) {
            (void)tp_adc_channel_push(adc, adc->block[i]);
        }
    } else {
        for (i = 0u; i < n; ++i) {
            (void)tp_adc_channel_push(adc, adc->block[i]);
        }
    }
    return TP_OK;
}

uint16_t tp_adc_channel_get_raw(const tp_adc_channel_t *adc)
{
    return (adc != NULL) ? adc->raw : 0u;
}

float tp_adc_channel_get_volts(const tp_adc_channel_t *adc)
{
    return (adc != NULL) ? adc->volts : 0.0f;
}

int tp_adc_channel_has_overrun(const tp_adc_channel_t *adc)
{
    return ((adc != NULL) && (adc->overruns > 0u)) ? 1 : 0;
}

void tp_adc_channel_clear_counters(tp_adc_channel_t *adc)
{
    if (adc == NULL) {
        return;
    }
    adc->overruns = 0u;
    adc->clamped = 0u;
}
