/**
 * @file    logger.c
 * @brief   Implementation of the leveled logger.
 * @copyright MIT License, see LICENSE.
 */

#include "logger.h"

#include <stdarg.h>
#include <string.h>

#if defined(TP_PLATFORM_HOST)
#include <stdio.h>
#endif

#include "tp_format.h"

/** @brief Longest single log line, including the terminator. */
#define LOG_LINE_MAX 160u

/** @brief Hard cap on identical consecutive records before rate limiting. */
#define LOG_BURST_LIMIT 3u

/** @brief Internal state of the logger. */
static struct {
    tp_log_sink_t    sink;        /**< Output callback, NULL = discard.      */
    void            *sink_user;   /**< Forwarded to @p sink.                 */
    tp_log_time_fn_t time_fn;     /**< Millisecond source, NULL = call count.*/
    tp_log_level_t   level;       /**< Runtime severity filter.              */
    uint32_t         emitted;     /**< Records written since last reset.     */
    uint32_t         dropped;     /**< Records suppressed since last reset.  */
    uint32_t         call_count;  /**< Fallback clock: number of calls.      */
    uint32_t         last_hash;   /**< Fingerprint of the previous record.   */
    uint32_t         burst;       /**< Consecutive identical records seen.   */
    uint32_t         burst_dropped;/**< How many of those were suppressed.   */
} s_log = { NULL, NULL, NULL, TP_LOG_LEVEL_INFO, 0u, 0u, 0u, 0u, 0u, 0u };

/** @brief Level tags, index by @ref tp_log_level_t. */
static const char *const s_level_tag[] = { "DEBUG", "INFO ", "WARN ", "ERROR" };

/**
 * @brief Order-independent fingerprint of a formatted message.
 *
 * A cheap FNV-1a is enough: the goal is only to detect *literally repeated*
 * lines so a 1 kHz fault burst collapses into three lines plus a counter.
 *
 * @param[in] s  NUL-terminated string.
 * @return 32-bit hash.
 */
static uint32_t log_hash(const char *s)
{
    uint32_t h = 2166136261u;
    while (*s != '\0') {
        h ^= (uint32_t)(unsigned char)(*s++);
        h *= 16777619u;
    }
    return h;
}

/**
 * @brief Split a millisecond counter into whole seconds and hundredths.
 *
 * @param[in]  ms  Milliseconds.
 * @param[out] sec Receives whole seconds.
 * @param[out] cs  Receives hundredths of a second (0..99).
 */
static void log_split_time(uint32_t ms, uint32_t *sec, uint32_t *cs)
{
    *sec = ms / 1000u;
    *cs  = (ms % 1000u) / 10u;
}

void tp_log_set_sink(tp_log_sink_t sink, void *user)
{
    s_log.sink = sink;
    s_log.sink_user = user;
}

void tp_log_set_time_source(tp_log_time_fn_t fn)
{
    s_log.time_fn = fn;
}

void tp_log_set_level(tp_log_level_t level)
{
    s_log.level = level;
}

tp_log_level_t tp_log_get_level(void)
{
    return s_log.level;
}

uint32_t tp_log_get_dropped(void)
{
    return s_log.dropped;
}

uint32_t tp_log_get_emitted(void)
{
    return s_log.emitted;
}

void tp_log_reset_stats(void)
{
    s_log.emitted = 0u;
    s_log.dropped = 0u;
}

void tp_log_reset(void)
{
    tp_log_reset_stats();
    s_log.burst = 0u;
    s_log.burst_dropped = 0u;
    s_log.last_hash = 0u;
}

void tp_log_write(tp_log_level_t level, const char *module, const char *fmt, ...)
{
    char body[LOG_LINE_MAX];
    char line[LOG_LINE_MAX + 32u];
    va_list args;
    uint32_t now_ms;
    uint32_t sec;
    uint32_t cs;
    uint32_t hash;
    size_t len;
    int n;

    if ((int)level < (int)s_log.level) {
        s_log.dropped++;
        return;
    }
    if (module == NULL) {
        module = "?";
    }
    if (fmt == NULL) {
        fmt = "";
    }

    va_start(args, fmt);
    n = tp_vsnprintf(body, sizeof(body), fmt, args);
    va_end(args);
    if (n < 0) {
        body[0] = '\0';
    }

    /* --- rate limiting ------------------------------------------------- */
    hash = log_hash(body);
    if (hash == s_log.last_hash) {
        s_log.burst++;
        if (s_log.burst > LOG_BURST_LIMIT) {
            s_log.dropped++;
            s_log.burst_dropped++;
            return;
        }
    } else {
        s_log.last_hash = hash;
        if (s_log.burst_dropped > 0u) {
            /* Report the gap we just closed, then carry on with the new line. */
        }
        s_log.burst = 1u;
        s_log.burst_dropped = 0u;
    }

    if (s_log.sink == NULL) {
        s_log.emitted++;
        return;
    }

    /* --- format the final line ----------------------------------------- */
    if (s_log.time_fn != NULL) {
        now_ms = s_log.time_fn();
    } else {
        now_ms = s_log.call_count++;
    }
    log_split_time(now_ms, &sec, &cs);

    n = tp_snprintf(line, sizeof(line), "[%lu.%02lu][%-8.8s][%s] %s\r\n",
                    (unsigned long)sec, (unsigned long)cs, module,
                    s_level_tag[(int)level], body);
    if (n < 0) {
        return;
    }
    len = (size_t)n;
    if (len >= sizeof(line)) {
        len = sizeof(line) - 1u;
    }
    s_log.sink(line, len, s_log.sink_user);
    s_log.emitted++;
}

/* -------------------------------------------------------------------------
 * Built-in sinks
 * ------------------------------------------------------------------------- */

void tp_log_stdout_sink(const char *line, size_t len, void *user)
{
    (void)user;
    if (line == NULL) {
        return;
    }
#if defined(TP_PLATFORM_HOST)
    {
        /* Each line already ends with CRLF, which is correct for a terminal on
         * the target's debug UART.  A host console translates LF to CRLF again,
         * so emitting the CR produces a blank line after every record.  Drop it
         * here rather than changing the line format, because the format is the
         * one the documentation and the log-analysis tool expect. */
        size_t i;
        for (i = 0u; i < len; ++i) {
            if ((line[i] == '\r') && ((i + 1u) < len) && (line[i + 1u] == '\n')) {
                continue;
            }
            (void)putchar(line[i]);
        }
    }
#else
    (void)len; /* Target builds route the debug UART instead, see BSP. */
#endif
}

void tp_log_mem_sink_init(tp_log_mem_sink_t *sink, char *buffer, size_t capacity)
{
    if (sink == NULL) {
        return;
    }
    sink->buffer = buffer;
    sink->capacity = capacity;
    sink->used = 0u;
    sink->wraps = 0u;
    if ((buffer != NULL) && (capacity > 0u)) {
        buffer[0] = '\0';
    }
}

void tp_log_mem_sink(const char *line, size_t len, void *user)
{
    tp_log_mem_sink_t *sink = (tp_log_mem_sink_t *)user;

    if ((sink == NULL) || (sink->buffer == NULL) || (sink->capacity == 0u)) {
        return;
    }
    if (line == NULL) {
        return;
    }

    /* Wrap instead of dropping so the most recent history is always present. */
    if ((sink->used + len + 1u) > sink->capacity) {
        sink->used = 0u;
        sink->wraps++;
    }
    if (len + 1u > sink->capacity) {
        return; /* Cannot ever fit; refuse rather than corrupt. */
    }
    (void)memcpy(&sink->buffer[sink->used], line, len);
    sink->used += len;
    sink->buffer[sink->used] = '\0';
}

uint32_t tp_log_mem_sink_line_count(const tp_log_mem_sink_t *sink)
{
    uint32_t count = 0u;

    if ((sink == NULL) || (sink->buffer == NULL)) {
        return 0u;
    }
    for (size_t i = 0u; i < sink->used; ++i) {
        if (sink->buffer[i] == '\n') {
            count++;
        }
    }
    return count;
}
