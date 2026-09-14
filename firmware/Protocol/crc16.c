/**
 * @file    crc16.c
 * @brief   Implementation of the CRC checksums.
 * @copyright MIT License, see LICENSE.
 */

#include "crc16.h"

uint16_t tp_crc16_modbus_byte(uint16_t crc, uint8_t byte)
{
    int i;

    /* Reflected algorithm: XOR the byte into the low half, then shift right and
     * apply the reversed polynomial (0xA001) whenever a one falls out of the
     * bottom.  This is bit-for-bit identical to the textbook MSB-first form
     * with the input and output reflected, which is what "CRC-16/MODBUS"
     * specifies. */
    crc ^= (uint16_t)byte;
    for (i = 0; i < 8; ++i) {
        if ((crc & 0x0001u) != 0u) {
            crc = (uint16_t)((crc >> 1) ^ 0xA001u);
        } else {
            crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

uint16_t tp_crc16_modbus_update(uint16_t crc, const uint8_t *data, size_t len)
{
    size_t i;

    if ((data == NULL) && (len > 0u)) {
        return crc;
    }
    for (i = 0u; i < len; ++i) {
        crc = tp_crc16_modbus_byte(crc, data[i]);
    }
    return crc;
}

uint16_t tp_crc16_modbus(const uint8_t *data, size_t len)
{
    return tp_crc16_modbus_update(TP_CRC16_MODBUS_INIT, data, len);
}

uint8_t tp_crc8_atm_update(uint8_t crc, const uint8_t *data, size_t len)
{
    size_t i;
    int    bit;

    if ((data == NULL) && (len > 0u)) {
        return crc;
    }
    for (i = 0u; i < len; ++i) {
        crc ^= data[i];
        for (bit = 0; bit < 8; ++bit) {
            /* MSB-first, polynomial 0x07: no reflection for CRC-8/ATM. */
            if ((crc & 0x80u) != 0u) {
                crc = (uint8_t)((uint8_t)(crc << 1) ^ 0x07u);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

uint8_t tp_crc8_atm(const uint8_t *data, size_t len)
{
    return tp_crc8_atm_update(0x00u, data, len);
}
