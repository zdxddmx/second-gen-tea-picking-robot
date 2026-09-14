/**
 * @file    tp_format.c
 * @brief   Implementation of the minimal embedded formatter.
 * @copyright MIT License, see LICENSE.
 */

#include "tp_format.h"

#include <stdint.h>

/** @brief Internal output cursor used while formatting. */
typedef struct {
    char  *buf;    /**< Destination buffer (may be NULL for a dry run).  */
    size_t cap;    /**< Capacity in bytes, including the terminator.     */
    size_t len;    /**< Characters produced so far (may exceed cap).     */
} out_t;

/**
 * @brief Append one character, honouring truncation.
 *
 * @param[in,out] o  Output cursor.
 * @param[in]     c  Character to append.
 */
static void out_char(out_t *o, char c)
{
    if ((o->buf != NULL) && (o->len + 1u < o->cap)) {
        o->buf[o->len] = c;
    }
    o->len++;
}

/**
 * @brief Append @p n copies of @p c (used for padding).
 *
 * @param[in,out] o  Output cursor.
 * @param[in]     c  Fill character.
 * @param[in]     n  Repeat count.
 */
static void out_fill(out_t *o, char c, int n)
{
    int i;
    for (i = 0; i < n; ++i) {
        out_char(o, c);
    }
}

/**
 * @brief Emit @p digits with optional zero padding, honouring width/left-align.
 *
 * @param[in,out] o        Output cursor.
 * @param[in]     text     Digit string, most-significant first.
 * @param[in]     ndigits  Length of @p text.
 * @param[in]     width    Minimum field width.
 * @param[in]     zero     Non-zero to pad with '0' (ignored when @p left).
 * @param[in]     left     Non-zero for '-' left justification.
 * @param[in]     prefix   Optional sign / `0x` prefix already consumed.
 * @param[in]     nprefix  Length of @p prefix.
 */
static void out_number(out_t *o, const char *text, int ndigits, int width,
                       int zero, int left, const char *prefix, int nprefix)
{
    int pad = width - ndigits - nprefix;
    int i;

    if (left) {
        for (i = 0; i < nprefix; ++i) {
            out_char(o, prefix[i]);
        }
        for (i = 0; i < ndigits; ++i) {
            out_char(o, text[i]);
        }
        out_fill(o, ' ', pad);
    } else if (zero) {
        /* Zero padding goes *after* the sign, i.e. "-007" not "00-7". */
        for (i = 0; i < nprefix; ++i) {
            out_char(o, prefix[i]);
        }
        out_fill(o, '0', pad);
        for (i = 0; i < ndigits; ++i) {
            out_char(o, text[i]);
        }
    } else {
        out_fill(o, ' ', pad);
        for (i = 0; i < nprefix; ++i) {
            out_char(o, prefix[i]);
        }
        for (i = 0; i < ndigits; ++i) {
            out_char(o, text[i]);
        }
    }
}

/**
 * @brief Convert an unsigned value to text in the requested base.
 *
 * @param[in]  value  Value to convert.
 * @param[in]  base   Numeric base, 10 or 16.
 * @param[in]  upper  Non-zero to use upper-case hex digits.
 * @param[out] buf    Receives the digits, most-significant first.
 * @return Number of digits written.
 */
static int u32_to_text(uint32_t value, uint32_t base, int upper, char *buf)
{
    static const char lower_digits[] = "0123456789abcdef";
    static const char upper_digits[] = "0123456789ABCDEF";
    const char *digits = upper ? upper_digits : lower_digits;
    char tmp[16];
    int n = 0;
    int i;

    if (value == 0u) {
        buf[0] = '0';
        return 1;
    }
    while ((value != 0u) && (n < (int)sizeof(tmp))) {
        tmp[n++] = digits[value % base];
        value /= base;
    }
    for (i = 0; i < n; ++i) {
        buf[i] = tmp[n - 1 - i];
    }
    return n;
}

int tp_format_float(char *dst, size_t size, float value, int decimals)
{
    char digits[40];
    char sign = 0;
    uint32_t whole;
    uint32_t frac;
    uint32_t scale = 1u;
    int ndigits;
    int i;
    int written = 0;

    if ((dst == NULL) || (size == 0u)) {
        return 0;
    }
    if (decimals < 0) {
        decimals = 0;
    }
    if (decimals > 6) {
        decimals = 6;
    }

    /* Non-finite values are reported explicitly instead of as garbage. */
    if (value != value) {
        const char *nan_text = "nan";
        for (i = 0; nan_text[i] != '\0'; ++i) {
            if ((size_t)written + 1u < size) {
                dst[written] = nan_text[i];
            }
            written++;
        }
        dst[(written < (int)size) ? written : ((int)size - 1)] = '\0';
        return written;
    }
    if ((value > 3.402823466e38f) || (value < -3.402823466e38f)) {
        const char *inf_text = (value > 0.0f) ? "inf" : "-inf";
        for (i = 0; inf_text[i] != '\0'; ++i) {
            if ((size_t)written + 1u < size) {
                dst[written] = inf_text[i];
            }
            written++;
        }
        dst[(written < (int)size) ? written : ((int)size - 1)] = '\0';
        return written;
    }

    if (value < 0.0f) {
        sign = '-';
        value = -value;
    }

    for (i = 0; i < decimals; ++i) {
        scale *= 10u;
    }

    /* Round half away from zero before splitting, so 1.995 -> "2.00". */
    {
        float scaled = (value * (float)scale) + 0.5f;
        uint32_t total;

        /* Guard against the float->uint32 conversion overflowing for large
         * magnitudes; fall back to a clamped representation. */
        if (scaled >= 4294967040.0f) {
            total = 0xFFFFFFFFu;
        } else {
            total = (uint32_t)scaled;
        }
        whole = total / scale;
        frac  = total % scale;
    }

    ndigits = 0;
    if (sign != 0) {
        digits[ndigits++] = sign;
    }
    {
        char wbuf[16];
        int wlen = u32_to_text(whole, 10u, 0, wbuf);
        for (i = 0; i < wlen; ++i) {
            digits[ndigits++] = wbuf[i];
        }
    }
    if (decimals > 0) {
        char fbuf[16];
        int flen;
        digits[ndigits++] = '.';
        /* Left-pad the fraction with zeros: 1.05 must not print as "1.5". */
        flen = u32_to_text(frac, 10u, 0, fbuf);
        for (i = flen; i < decimals; ++i) {
            digits[ndigits++] = '0';
        }
        for (i = 0; i < flen; ++i) {
            digits[ndigits++] = fbuf[i];
        }
    }

    for (i = 0; i < ndigits; ++i) {
        if ((size_t)written + 1u < size) {
            dst[written] = digits[i];
        }
        written++;
    }
    dst[(written < (int)size) ? written : ((int)size - 1)] = '\0';
    return written;
}

int tp_vsnprintf(char *dst, size_t size, const char *fmt, va_list args)
{
    out_t o;
    int i;

    if (dst == NULL) {
        return 0;
    }
    if (size == 0u) {
        return 0;
    }
    o.buf = dst;
    o.cap = size;
    o.len = 0u;

    if (fmt == NULL) {
        dst[0] = '\0';
        return 0;
    }

    for (i = 0; fmt[i] != '\0'; ++i) {
        int left = 0;
        int zero = 0;
        int width = 0;
        int prec = -1;
        int is_long = 0;
        char conv;

        if (fmt[i] != '%') {
            out_char(&o, fmt[i]);
            continue;
        }

        /* --- flags ------------------------------------------------------ */
        for (;;) {
            if (fmt[i + 1] == '-')      { left = 1; i++; }
            else if (fmt[i + 1] == '0') { zero = 1; i++; }
            else if (fmt[i + 1] == '+') { i++; } /* accepted, ignored */
            else if (fmt[i + 1] == ' ') { i++; } /* accepted, ignored */
            else { break; }
        }
        /* --- width ------------------------------------------------------ */
        if (fmt[i + 1] == '*') {
            width = va_arg(args, int);
            i++;
            if (width < 0) {
                left = 1;
                width = -width;
            }
        } else {
            while ((fmt[i + 1] >= '0') && (fmt[i + 1] <= '9')) {
                width = (width * 10) + (fmt[i + 1] - '0');
                i++;
            }
        }
        /* --- precision -------------------------------------------------- */
        if (fmt[i + 1] == '.') {
            i++;
            prec = 0;
            while ((fmt[i + 1] >= '0') && (fmt[i + 1] <= '9')) {
                prec = (prec * 10) + (fmt[i + 1] - '0');
                i++;
            }
        }
        /* --- length ----------------------------------------------------- */
        while ((fmt[i + 1] == 'l') || (fmt[i + 1] == 'h') || (fmt[i + 1] == 'z')) {
            if (fmt[i + 1] == 'l') {
                is_long = 1;
            }
            i++;
        }

        conv = fmt[i + 1];
        if (conv == '\0') {
            out_char(&o, '%');
            break;
        }
        i++;

        switch (conv) {
        case '%':
            out_char(&o, '%');
            break;

        case 'c':
            out_char(&o, (char)va_arg(args, int));
            break;

        case 's': {
            const char *s = va_arg(args, const char *);
            int slen = 0;
            int k;
            if (s == NULL) {
                s = "(null)";
            }
            while (s[slen] != '\0') {
                slen++;
            }
            if ((prec >= 0) && (slen > prec)) {
                slen = prec;
            }
            if (!left) {
                out_fill(&o, ' ', width - slen);
            }
            for (k = 0; k < slen; ++k) {
                out_char(&o, s[k]);
            }
            if (left) {
                out_fill(&o, ' ', width - slen);
            }
            break;
        }

        case 'd':
        case 'i': {
            int32_t v = is_long ? (int32_t)va_arg(args, long)
                                : (int32_t)va_arg(args, int);
            char text[16];
            char prefix[2];
            uint32_t mag;
            int npref = 0;
            int nd;
            if (v < 0) {
                prefix[npref++] = '-';
                /* Negate in unsigned space so INT32_MIN is handled correctly. */
                mag = (uint32_t)(-(int64_t)v);
            } else {
                mag = (uint32_t)v;
            }
            nd = u32_to_text(mag, 10u, 0, text);
            out_number(&o, text, nd, width, zero, left, prefix, npref);
            break;
        }

        case 'u': {
            uint32_t v = is_long ? (uint32_t)va_arg(args, unsigned long)
                                 : va_arg(args, uint32_t);
            char text[16];
            int nd = u32_to_text(v, 10u, 0, text);
            out_number(&o, text, nd, width, zero, left, NULL, 0);
            break;
        }

        case 'x':
        case 'X': {
            uint32_t v = is_long ? (uint32_t)va_arg(args, unsigned long)
                                 : va_arg(args, uint32_t);
            char digits_buf[24];
            int upper = (conv == 'X') ? 1 : 0;
            int nd = u32_to_text(v, 16u, upper, digits_buf);
            int pad;

            /* `%08X` is the common case in this firmware (CAN IDs, register
             * dumps), so a precision widens the digit string with zeros before
             * the width/justification logic runs. */
            if ((prec > 0) && (prec > nd) && (prec <= 16)) {
                pad = prec - nd;
                for (int m = nd - 1; m >= 0; --m) {
                    digits_buf[pad + m] = digits_buf[m];
                }
                for (int m = 0; m < pad; ++m) {
                    digits_buf[m] = '0';
                }
                nd = prec;
            }
            out_number(&o, digits_buf, nd, width, zero, left, NULL, 0);
            break;
        }

        case 'f':
        case 'F': {
            /* `float` is promoted to `double` in varargs; on Cortex-M4 with a
             * single-precision FPU the compiler still passes a double, so we
             * must read a double here for the ABI to match. */
            double dv = va_arg(args, double);
            char text[32];
            int nd = tp_format_float(text, sizeof(text), (float)dv,
                                     (prec >= 0) ? prec : 3);
            /* tp_format_float already returns the character count *excluding*
             * the terminator, so it is used as-is: subtracting one here silently
             * truncated the last digit of every float, which turned "3000" into
             * "300" and "0.0" into "0.". */
            out_number(&o, text, nd, width, zero, left, NULL, 0);
            break;
        }

        default:
            /* Unknown conversion: emit it literally instead of mis-reading the
             * argument list, which would corrupt every following field. */
            out_char(&o, '%');
            out_char(&o, conv);
            break;
        }
    }

    /* Always terminate. */
    if (o.len < o.cap) {
        o.buf[o.len] = '\0';
    } else {
        o.buf[o.cap - 1u] = '\0';
    }
    return (int)o.len;
}

int tp_snprintf(char *dst, size_t size, const char *fmt, ...)
{
    va_list args;
    int n;

    va_start(args, fmt);
    n = tp_vsnprintf(dst, size, fmt, args);
    va_end(args);
    return n;
}
