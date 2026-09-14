/**
 * @file    protocol_codec.c
 * @brief   Little-endian field codecs and fixed-point conversions.
 * @copyright MIT License, see LICENSE.
 */

#include "protocol_types.h"

#include "compiler.h"

void tp_le_put_i16(uint8_t *out, int16_t v)
{
    if (out == NULL) {
        return;
    }
    out[0] = (uint8_t)((uint16_t)v & 0x00FFu);
    out[1] = (uint8_t)(((uint16_t)v >> 8) & 0x00FFu);
}

void tp_le_put_u16(uint8_t *out, uint16_t v)
{
    if (out == NULL) {
        return;
    }
    out[0] = (uint8_t)(v & 0x00FFu);
    out[1] = (uint8_t)((v >> 8) & 0x00FFu);
}

void tp_le_put_i32(uint8_t *out, int32_t v)
{
    if (out == NULL) {
        return;
    }
    out[0] = (uint8_t)((uint32_t)v & 0xFFu);
    out[1] = (uint8_t)(((uint32_t)v >> 8) & 0xFFu);
    out[2] = (uint8_t)(((uint32_t)v >> 16) & 0xFFu);
    out[3] = (uint8_t)(((uint32_t)v >> 24) & 0xFFu);
}

void tp_le_put_u32(uint8_t *out, uint32_t v)
{
    if (out == NULL) {
        return;
    }
    out[0] = (uint8_t)(v & 0xFFu);
    out[1] = (uint8_t)((v >> 8) & 0xFFu);
    out[2] = (uint8_t)((v >> 16) & 0xFFu);
    out[3] = (uint8_t)((v >> 24) & 0xFFu);
}

int16_t tp_le_get_i16(const uint8_t *in)
{
    if (in == NULL) {
        return 0;
    }
    return (int16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

uint16_t tp_le_get_u16(const uint8_t *in)
{
    if (in == NULL) {
        return 0u;
    }
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

int32_t tp_le_get_i32(const uint8_t *in)
{
    if (in == NULL) {
        return 0;
    }
    return (int32_t)((uint32_t)in[0] | ((uint32_t)in[1] << 8)
                   | ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24));
}

uint32_t tp_le_get_u32(const uint8_t *in)
{
    if (in == NULL) {
        return 0u;
    }
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8)
         | ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
}

int32_t tp_fixed_from_float(float value, float scale, int32_t min_out,
                            int32_t max_out)
{
    float scaled;

    if (!TP_ISFINITE(value) || !TP_ISFINITE(scale) || (scale <= 0.0f)) {
        /* A non-finite value must never reach the wire as an integer: the cast
         * of a NaN is undefined and in practice produces INT32_MIN, which the
         * far end would read as a huge negative force.  Return the low bound
         * instead, which is at least a legal, reportable value. */
        return min_out;
    }
    scaled = value * scale;
    /* Round half away from zero before clamping, so the encoder and decoder
     * round-trip exactly for values that are representable. */
    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;

    if (scaled >= (float)max_out) {
        return max_out;
    }
    if (scaled <= (float)min_out) {
        return min_out;
    }
    return (int32_t)scaled;
}

float tp_fixed_to_float(int32_t value, float scale)
{
    if (!TP_ISFINITE(scale) || (scale <= 0.0f)) {
        return 0.0f;
    }
    return (float)value / scale;
}
