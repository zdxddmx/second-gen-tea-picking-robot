/**
 * @file    status.h
 * @brief   Uniform return-code type shared by every module of the firmware.
 *
 * @details
 * Why a dedicated status type instead of `int`?
 *
 *   - `int` return codes rot: nobody remembers whether 1 means success or
 *     failure, and negative/positive conventions drift between modules.
 *   - A single enumerator makes the *set* of possible failures visible in the
 *     header, which is exactly what a reviewer (or an interviewer) wants to
 *     inspect.
 *   - It keeps the door open for a `switch` over the status in the fault
 *     manager, so every failure mode has to be handled explicitly.
 *
 * Convention: `TP_OK` is 0 and every error is negative, mirroring POSIX, so
 * `if (status < 0)` is a valid catch-all test.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_STATUS_H
#define TP_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Result of an operation that can fail.
 */
typedef enum {
    TP_OK               =  0, /**< Success.                              */
    TP_ERR_PARAM        = -1, /**< NULL pointer or invalid argument.      */
    TP_ERR_STATE        = -2, /**< Operation illegal in the current state.*/
    TP_ERR_TIMEOUT      = -3, /**< Deadline expired.                      */
    TP_ERR_CRC          = -4, /**< Checksum mismatch on a received frame. */
    TP_ERR_RANGE        = -5, /**< Value outside the permitted range.     */
    TP_ERR_NO_MEM       = -6, /**< Allocation failed.                     */
    TP_ERR_NOT_READY    = -7, /**< Resource not initialised / not homed.  */
    TP_ERR_UNSUPPORTED  = -8, /**< Feature not available on this target.  */
    TP_ERR_IO           = -9, /**< Peripheral-level failure.              */
    TP_ERR_FAULT_ACTIVE = -10,/**< A latched fault forbids the operation. */
    TP_ERR_BUSY         = -11,/**< Resource currently owned by someone.   */
    TP_ERR_OVERFLOW     = -12,/**< Buffer/queue full, numeric overflow.   */
    TP_ERR_NOT_FOUND    = -13,/**< Lookup missed.                         */
    TP_ERR_PROTOCOL     = -14,/**< Malformed frame, unexpected command.   */
    TP_ERR_DIVERGED     = -15 /**< Numeric result non-finite.             */
} tp_status_t;

/**
 * @brief Human-readable name of a status code.
 *
 * @param[in] status  Status code to describe.
 * @return A static, never-NULL string such as `"TP_ERR_CRC"`.  Unknown codes
 *         return `"TP_ERR_UNKNOWN"`; the caller must not free the pointer.
 *
 * @note Deliberately returns a `const char *` into `.rodata` rather than
 *       filling a caller buffer, so it is safe to call from an ISR or a
 *       logging path with no stack budget.
 */
const char *tp_status_str(tp_status_t status);

/**
 * @brief Short lowercase description suitable for a log message tail.
 *
 * @param[in] status  Status code to describe.
 * @return A static string such as `"crc mismatch"`.
 */
const char *tp_status_desc(tp_status_t status);

/**
 * @brief Whether a status represents a failure.
 *
 * @param[in] status  Status code to test.
 * @return `true` when @p status is not @ref TP_OK.
 */
static inline int tp_status_failed(tp_status_t status)
{
    return (status != TP_OK) ? 1 : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* TP_STATUS_H */
