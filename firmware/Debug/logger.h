/**
 * @file    logger.h
 * @brief   Leveled, timestamped logging with pluggable output sinks.
 *
 * @details
 * The log format is fixed by design so that a log excerpt can be pasted
 * straight into a report or a bug ticket and still be readable:
 *
 * @verbatim
 * [1234.52][CONTROL  ][INFO ] Target position received: 12500 mrad
 * [1234.72][GRIPPER  ][INFO ] Contact detected at gap 6.03 mm
 * [1234.81][SAFETY   ][WARN ] Force 3.42 N approaching soft limit 3.50 N
 * [1235.10][SAFETY   ][ERROR] ERR_FORCE_OVERLOAD (code 1), motor disabled
 * @endverbatim
 *
 * Design notes:
 *
 *   - **Timestamp source is injectable.**  @ref tp_log_set_time_source lets a
 *     unit test drive a fake clock, so a test can assert on an exact log line
 *     instead of a moving one.  On the target the default is the OS tick.
 *   - **Sink is injectable and may be NULL.**  On the target the sink is the
 *     debug UART behind a DMA ring; on the host it is `stdout`; in a unit test
 *     it is a callback that appends to a RAM buffer.  A NULL sink means "log
 *     nowhere", which is how the debug task is disabled in flight.
 *   - **Never blocks the caller unboundedly.**  The sink is required to be
 *     non-blocking (queue a DMA transfer, write to a memory buffer).  A logger
 *     that can stall the 1 kHz control loop is a real-time bug.
 *   - **Rate limiting with an explicit drop counter.**  A fault that fires
 *     every millisecond would otherwise flood the link and hide the first
 *     occurrence, which is the only one that matters.  Suppressed lines are
 *     counted and reported by @ref tp_log_get_dropped.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_LOGGER_H
#define TP_LOGGER_H

#include <stddef.h>
#include <stdint.h>

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Severity of a log record. */
typedef enum {
    TP_LOG_DEBUG = TP_LOG_LEVEL_DEBUG, /**< Loop-level tracing, off by default.*/
    TP_LOG_INFO  = TP_LOG_LEVEL_INFO,  /**< Normal state transitions.         */
    TP_LOG_WARN  = TP_LOG_LEVEL_WARN,  /**< Recoverable anomaly.              */
    TP_LOG_ERROR = TP_LOG_LEVEL_ERROR  /**< Fault, motor disabled.            */
} tp_log_level_t;

/**
 * @brief Receives one fully formatted log line.
 *
 * @param[in] line  NUL-terminated line, already includes `[ts][mod][lvl]`.
 * @param[in] len   Length of @p line excluding the terminator.
 * @param[in] user  Opaque pointer supplied to @ref tp_log_set_sink.
 *
 * @warning Must not block and must not call back into the logger.
 */
typedef void (*tp_log_sink_t)(const char *line, size_t len, void *user);

/**
 * @brief Millisecond time source used for the timestamp field.
 *
 * @return Free-running millisecond counter (wraps at 2^32, i.e. ~49.7 days).
 */
typedef uint32_t (*tp_log_time_fn_t)(void);

/**
 * @brief Install the output sink.
 *
 * @param[in] sink  Callback, or NULL to discard every record.
 * @param[in] user  Opaque pointer forwarded to @p sink.
 */
void tp_log_set_sink(tp_log_sink_t sink, void *user);

/**
 * @brief Install the millisecond time source.
 *
 * @param[in] fn  Callback, or NULL to fall back to an internal counter that
 *                only counts calls (useful in pure unit tests).
 */
void tp_log_set_time_source(tp_log_time_fn_t fn);

/**
 * @brief Set the runtime severity filter.
 *
 * @param[in] level  Records below this level are dropped and counted.
 */
void tp_log_set_level(tp_log_level_t level);

/**
 * @brief Read the runtime severity filter.
 *
 * @return The currently active minimum level.
 */
tp_log_level_t tp_log_get_level(void);

/**
 * @brief Format and emit one log record.
 *
 * Prefer the @ref LOG_DEBUG / @ref LOG_INFO / @ref LOG_WARN / @ref LOG_ERROR
 * macros over calling this directly: they let the @c TP_LOG_LEVEL build-time
 * filter remove the call entirely from a release image.
 *
 * @param[in] level   Severity.
 * @param[in] module  Short module tag, 8 characters or fewer, e.g. `"CONTROL"`.
 * @param[in] fmt     Format string understood by @ref tp_format.h.
 * @param[in] ...     Format arguments.
 */
void tp_log_write(tp_log_level_t level, const char *module, const char *fmt, ...);

/**
 * @brief Number of records suppressed by the rate limiter.
 * @return Count since the last @ref tp_log_reset_stats.
 */
uint32_t tp_log_get_dropped(void);

/**
 * @brief Number of records actually emitted.
 * @return Count since the last @ref tp_log_reset_stats.
 */
uint32_t tp_log_get_emitted(void);

/** @brief Clear the emitted/dropped counters. */
void tp_log_reset_stats(void);

/** @brief Clear the emitted/dropped counters (alias kept for symmetry). */
void tp_log_reset(void);

/* -------------------------------------------------------------------------
 * Hardware-free convenience sink
 * ------------------------------------------------------------------------- */

/**
 * @brief Built-in sink that writes to `stdout`.
 *
 * Provided so a host build and the unit tests need no boilerplate: call
 * `tp_log_set_sink(tp_log_stdout_sink, NULL);` and logging works.  On the
 * STM32 target this sink is compiled out (`TP_PLATFORM_HOST` guard) because
 * there is no `stdout`.
 *
 * @param[in] line  Formatted line.
 * @param[in] len   Length excluding the terminator.
 * @param[in] user  Ignored.
 */
void tp_log_stdout_sink(const char *line, size_t len, void *user);

/**
 * @brief Built-in sink that appends into a caller-owned RAM buffer.
 *
 * Useful for a "last N lines" black box that survives a reset in a
 * no-init RAM section, and for asserting on log content in unit tests.
 *
 * @param[in] line  Formatted line.
 * @param[in] len   Length excluding the terminator.
 * @param[in] user  Pointer to a @ref tp_log_mem_sink_t.
 */
void tp_log_mem_sink(const char *line, size_t len, void *user);

/** @brief State of the RAM-buffer sink, see @ref tp_log_mem_sink. */
typedef struct {
    char  *buffer;  /**< Storage owned by the caller.                     */
    size_t capacity;/**< Size of @p buffer in bytes.                      */
    size_t used;    /**< Bytes currently occupied.                        */
    uint32_t wraps; /**< How many times the buffer wrapped around.        */
} tp_log_mem_sink_t;

/**
 * @brief Initialise a RAM-buffer sink.
 *
 * @param[in,out] sink      Sink state to initialise.
 * @param[in]     buffer    Caller-owned storage.
 * @param[in]     capacity  Size of @p buffer in bytes.
 */
void tp_log_mem_sink_init(tp_log_mem_sink_t *sink, char *buffer, size_t capacity);

/**
 * @brief Number of complete lines currently held by a RAM-buffer sink.
 *
 * @param[in] sink  Initialised sink.
 * @return Line count.
 */
uint32_t tp_log_mem_sink_line_count(const tp_log_mem_sink_t *sink);

/* -------------------------------------------------------------------------
 * Logging macros
 * ------------------------------------------------------------------------- */

#if (TP_LOG_LEVEL <= TP_LOG_LEVEL_DEBUG)
#define LOG_DEBUG(module, ...) tp_log_write(TP_LOG_DEBUG, (module), __VA_ARGS__)
#else
#define LOG_DEBUG(module, ...) ((void)0)
#endif

#if (TP_LOG_LEVEL <= TP_LOG_LEVEL_INFO)
#define LOG_INFO(module, ...)  tp_log_write(TP_LOG_INFO, (module), __VA_ARGS__)
#else
#define LOG_INFO(module, ...)  ((void)0)
#endif

#if (TP_LOG_LEVEL <= TP_LOG_LEVEL_WARN)
#define LOG_WARN(module, ...)  tp_log_write(TP_LOG_WARN, (module), __VA_ARGS__)
#else
#define LOG_WARN(module, ...)  ((void)0)
#endif

#if (TP_LOG_LEVEL <= TP_LOG_LEVEL_ERROR)
#define LOG_ERROR(module, ...) tp_log_write(TP_LOG_ERROR, (module), __VA_ARGS__)
#else
#define LOG_ERROR(module, ...) ((void)0)
#endif

/** @brief Module tags, kept in one place so log greps stay stable. */
#define TP_MOD_MAIN    "MAIN"
#define TP_MOD_CONTROL "CONTROL"
#define TP_MOD_SENSOR  "SENSOR"
#define TP_MOD_COMM    "COMM"
#define TP_MOD_ROBOT   "ROBOT"
#define TP_MOD_SAFETY  "SAFETY"
#define TP_MOD_GRIPPER "GRIPPER"
#define TP_MOD_MOTOR   "MOTOR"
#define TP_MOD_ENCODER "ENCODER"
#define TP_MOD_FORCE   "FORCE"
#define TP_MOD_PROTO   "PROTO"
#define TP_MOD_TELEM   "TELEMETRY"
#define TP_MOD_DEBUG   "DEBUG"
#define TP_MOD_TEST    "TEST"

#ifdef __cplusplus
}
#endif

#endif /* TP_LOGGER_H */
