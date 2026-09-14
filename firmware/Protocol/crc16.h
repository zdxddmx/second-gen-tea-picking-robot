/**
 * @file    crc16.h
 * @brief   CRC-16/MODBUS and CRC-8 checksums used by the wire protocols.
 *
 * @details
 * # Why CRC-16/MODBUS specifically
 * The robot speaks two protocols and both need a checksum.  Rather than
 * inventing one, this project uses **CRC-16/MODBUS** (polynomial `0x8005`,
 * reflected as `0xA001`, init `0xFFFF`, no final XOR, reflected in and out)
 * because:
 *
 *   - Modbus RTU mandates it, so the RS485 link gets its checksum for free;
 *   - it is catalogued (`CRC-16/MODBUS`) so an interoperability partner can
 *     verify an implementation against the published check value without
 *     reading this code;
 *   - its 16-bit width gives a probability of `2^-16` of accepting a corrupted
 *     frame, and it detects all single-bit errors, all odd-bit errors, and all
 *     burst errors up to 16 bits long.  That last property is the one that
 *     matters on a machine with a PWM drive: interference produces *bursts*,
 *     not isolated bit flips.
 *
 * # Bitwise versus table-driven
 * The bitwise implementation below runs at roughly 8 shift/XOR rounds per byte.
 * At 921600 baud the link delivers ~92 kbyte/s worst case, i.e. about 736 k
 * inner iterations per second, or well under 1 % of a 168 MHz Cortex-M4.  A
 * 512-byte lookup table would cut that to a few percent of one percent, at the
 * cost of 512 bytes of flash that is never needed.  **A table was therefore
 * deliberately not added**; `docs/13_future_work.md` records the exact
 * condition under which that decision should be revisited (sustained traffic
 * above roughly 5 Mbaud, or a tighter CPU budget).
 *
 * # CRC-8 is used for what
 * The CAN auxiliary frames are 8 bytes total, so a 16-bit checksum would eat a
 * quarter of the payload.  CRC-8/ATM (`poly 0x07`, init `0x00`) gives adequate
 * protection for a link that already has a 15-bit CRC at the data-link layer;
 * its purpose is to catch *application-level* mistakes (a wrong scaling factor,
 * a stale struct), not bit errors.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_CRC16_H
#define TP_CRC16_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Initial value mandated by CRC-16/MODBUS. */
#define TP_CRC16_MODBUS_INIT 0xFFFFu

/** @brief Check value for CRC-16/MODBUS over the ASCII string "123456789". */
#define TP_CRC16_MODBUS_CHECK 0x4B37u

/**
 * @brief Compute CRC-16/MODBUS over a buffer.
 *
 * @param[in] data  Bytes to checksum.  May be NULL only when @p len is 0.
 * @param[in] len   Number of bytes.
 * @return 16-bit checksum.
 */
uint16_t tp_crc16_modbus(const uint8_t *data, size_t len);

/**
 * @brief Continue a CRC computation over a second buffer.
 *
 * Needed because the frame checksum covers `CMD | LEN | PAYLOAD`, which are
 * assembled in two different places; recomputing over a concatenated copy would
 * need a second buffer.
 *
 * @param[in] crc   CRC accumulated so far, or @ref TP_CRC16_MODBUS_INIT.
 * @param[in] data  Additional bytes.
 * @param[in] len   Number of bytes.
 * @return Updated checksum.
 */
uint16_t tp_crc16_modbus_update(uint16_t crc, const uint8_t *data, size_t len);

/**
 * @brief Compute a checksum over a single byte.
 *
 * @param[in] crc   CRC accumulated so far.
 * @param[in] byte  Byte to fold in.
 * @return Updated checksum.
 */
uint16_t tp_crc16_modbus_byte(uint16_t crc, uint8_t byte);

/**
 * @brief Compute CRC-8/ATM over a buffer.
 *
 * @param[in] data  Bytes to checksum.
 * @param[in] len   Number of bytes.
 * @return 8-bit checksum.
 */
uint8_t tp_crc8_atm(const uint8_t *data, size_t len);

/**
 * @brief Continue a CRC-8/ATM computation.
 *
 * @param[in] crc   CRC accumulated so far.
 * @param[in] data  Additional bytes.
 * @param[in] len   Number of bytes.
 * @return Updated checksum.
 */
uint8_t tp_crc8_atm_update(uint8_t crc, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* TP_CRC16_H */
