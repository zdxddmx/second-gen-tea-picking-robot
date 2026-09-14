/**
 * @file    adc_driver.h
 * @brief   ADC acquisition front end: DMA blocks, scaling, overrun accounting.
 *
 * @details
 * # Why DMA and not polled conversions
 * Five analogue channels are needed (force, two currents, bus voltage, plus a
 * spare) and the sensor task runs at 500 Hz.  Polled single conversions would
 * mean five blocking conversions per period; at the STM32F407's fastest
 * 12-bit conversion time (about 0.5 us) that is only 2.5 us of CPU, so the
 * *time* is not the argument.  The arguments are:
 *
 *   1. **Jitter.**  With polling, the sampling instant depends on when the
 *      sensor task happens to run.  With a timer-triggered circular DMA, the
 *      sample instants are locked to a hardware timer and are therefore
 *      *equally spaced*.  The velocity estimator divides by elapsed time and
 *      the force controller differentiates; both degrade badly with jitter.
 *   2. **Coherence.**  A DMA block captures all channels within microseconds
 *      of each other, so the force and the current in one record really belong
 *      to the same instant.  Polled reads sample them up to a millisecond
 *      apart, and any correlation analysis between them is then meaningless.
 *   3. **CPU availability.**  The transfer costs zero cycles, which leaves the
 *      1 kHz control task undisturbed.
 *
 * # Why the buffer is deliberately small
 * The DMA writes into a two-half circular buffer of
 * `TP_ADC_DMA_BUFFER_LEN` samples per channel.  The sensor task must consume
 * each half before the DMA wraps back onto it; the overrun counter in
 * @ref tp_adc_channel_t::overruns makes a missed deadline visible instead of
 * silently corrupting the data.  Sizing the buffer is a real trade-off: too
 * small and a scheduling hiccup loses data, too large and the latency between
 * sampling and consumption grows.  16 samples at 500 Hz is 32 ms of slack,
 * which comfortably covers a FreeRTOS tick plus a flash-erase-sized hiccup.
 *
 * # Averaging and decimation
 * @ref tp_adc_channel_t::decimation lets the caller trade bandwidth for noise
 * without touching the filter chain: averaging 4 samples of a 2 kHz DMA stream
 * gives a 500 Hz output with 6 dB of extra noise rejection that costs nothing
 * in phase lag compared with an IIR filter of the same attenuation.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_ADC_DRIVER_H
#define TP_ADC_DRIVER_H

#include <stdint.h>

#include "bsp_types.h"
#include "filters.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Samples per channel in the DMA circular buffer. */
#define TP_ADC_BLOCK_MAX 32u

/**
 * @brief One analogue channel.
 */
typedef struct {
    tp_adc_ch_t ch;          /**< Hardware channel.                          */
    float    vref_v;         /**< Reference voltage.                         */
    float    full_scale;     /**< Maximum count value (4095 for 12 bit).     */
    float    divider;        /**< External divider ratio; 1.0 for direct.    */
    uint16_t raw;            /**< Latest raw conversion.                     */
    uint16_t block[TP_ADC_BLOCK_MAX]; /**< Latest DMA block.                  */
    uint32_t block_len;      /**< Samples in @ref block.                     */
    uint32_t decimation;     /**< Samples averaged into @ref raw.            */
    uint32_t pending;        /**< Samples accumulated towards decimation.    */
    uint32_t pending_sum;    /**< Running sum for the decimation average.    */
    float    volts;          /**< Latest reading in volts, divider-corrected.*/
    uint32_t sample_count;   /**< Samples consumed.                          */
    uint32_t overruns;       /**< Blocks missed because the caller was late. */
    uint32_t clamped;        /**< Blocks clamped because the input pegged.   */
    int      initialised;    /**< Non-zero after a successful init.          */
} tp_adc_channel_t;

/**
 * @brief Initialise an ADC channel.
 *
 * @param[in,out] adc         Instance.
 * @param[in]     ch          Hardware channel.
 * @param[in]     dt_s        Consumer period in seconds; sets the default
 *                            decimation so the effective rate matches.
 * @param[in]     divider     External divider ratio (e.g. 10.0 for the 24 V
 *                            rail), or 1.0 when the signal is direct.
 * @param[in]     dma_rate_hz Rate at which the DMA stream produces samples.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_adc_channel_init(tp_adc_channel_t *adc, tp_adc_ch_t ch, float dt_s,
                                float divider, uint32_t dma_rate_hz);

/**
 * @brief Service one consumer period: drain the DMA block and update the value.
 *
 * @param[in,out] adc  Instance.
 * @return @ref TP_OK, @ref TP_ERR_NOT_READY when no block was ready (counted as
 *         an overrun), or @ref TP_ERR_PARAM.
 */
tp_status_t tp_adc_channel_service(tp_adc_channel_t *adc);

/**
 * @brief Feed a raw sample directly, bypassing the DMA path.
 *
 * Used by the host test harness, which has no DMA and drives the channel one
 * sample at a time.  Keeping this as a public function rather than a test-only
 * hack means the decimation logic is exercised by the same code on both
 * targets.
 *
 * @param[in,out] adc    Instance.
 * @param[in]     counts Raw value.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_adc_channel_push(tp_adc_channel_t *adc, uint16_t counts);

/**
 * @brief Convert counts to volts using this channel's reference and divider.
 *
 * @param[in] adc     Instance.
 * @param[in] counts  Raw value.
 * @return Voltage at the signal source.
 */
float tp_adc_channel_counts_to_volts(const tp_adc_channel_t *adc, uint16_t counts);

/**
 * @brief Latest decimated raw value.
 *
 * @param[in] adc  Instance.
 * @return Raw count value.
 */
uint16_t tp_adc_channel_get_raw(const tp_adc_channel_t *adc);

/**
 * @brief Latest value in volts.
 *
 * @param[in] adc  Instance.
 * @return Voltage at the source.
 */
float tp_adc_channel_get_volts(const tp_adc_channel_t *adc);

/**
 * @brief Whether the channel has reported any overrun.
 *
 * @param[in] adc  Instance.
 * @return Non-zero when at least one block was missed.
 */
int tp_adc_channel_has_overrun(const tp_adc_channel_t *adc);

/**
 * @brief Clear the overrun and clamp counters.
 *
 * @param[in,out] adc  Instance.
 */
void tp_adc_channel_clear_counters(tp_adc_channel_t *adc);

#ifdef __cplusplus
}
#endif

#endif /* TP_ADC_DRIVER_H */
