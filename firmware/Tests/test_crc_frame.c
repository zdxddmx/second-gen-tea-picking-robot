/**
 * @file    test_crc_frame.c
 * @brief   Unit tests for the CRC and frame codec.
 * @copyright MIT License, see LICENSE.
 */

#include "test_framework.h"

#include "crc16.h"
#include "frame.h"

/* =========================================================================
 * CRC
 * ========================================================================= */

TP_TEST_CASE(crc, modbus_check_vector)
{
    /* The published check value for CRC-16/MODBUS over "123456789".  If this
     * fails, nothing else in the protocol layer can be trusted, and an
     * interoperability partner would compute a different checksum. */
    const uint8_t vector[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    TP_ASSERT_EQ(tp_crc16_modbus(vector, sizeof(vector)),
                 (uint16_t)TP_CRC16_MODBUS_CHECK);
}

TP_TEST_CASE(crc, empty_buffer_returns_init)
{
    TP_ASSERT_EQ(tp_crc16_modbus(NULL, 0u), (uint16_t)TP_CRC16_MODBUS_INIT);
    {
        const uint8_t one = 0x00u;
        TP_ASSERT_EQ(tp_crc16_modbus(&one, 1u),
                     tp_crc16_modbus_byte(TP_CRC16_MODBUS_INIT, 0x00u));
    }
}

TP_TEST_CASE(crc, incremental_matches_one_shot)
{
    const uint8_t a[] = { 0x10u, 0x02u };
    const uint8_t b[] = { 0x11u, 0x22u, 0x33u };
    uint8_t joined[5];
    uint16_t incremental;

    (void)memcpy(joined, a, sizeof(a));
    (void)memcpy(&joined[2], b, sizeof(b));

    incremental = tp_crc16_modbus_update(TP_CRC16_MODBUS_INIT, a, sizeof(a));
    incremental = tp_crc16_modbus_update(incremental, b, sizeof(b));

    /* The protocol computes the CRC over CMD | LEN | PAYLOAD, which are built in
     * two different places, so the incremental form has to agree exactly. */
    TP_ASSERT_EQ(incremental, tp_crc16_modbus(joined, sizeof(joined)));
}

TP_TEST_CASE(crc, single_bit_errors_are_always_detected)
{
    uint8_t data[8] = { 0xAAu, 0x55u, 0x10u, 0x04u, 1u, 2u, 3u, 4u };
    uint16_t base = tp_crc16_modbus(data, sizeof(data));
    unsigned i;
    unsigned bit;

    /* Every single-bit error must change the checksum.  This is a defining
     * property of a CRC, and asserting it catches a wrong polynomial or a
     * missing final reflection far more reliably than one check vector does. */
    for (i = 0u; i < sizeof(data); ++i) {
        for (bit = 0u; bit < 8u; ++bit) {
            uint16_t changed;
            data[i] ^= (uint8_t)(1u << bit);
            changed = tp_crc16_modbus(data, sizeof(data));
            TP_ASSERT_NE(changed, base);
            data[i] ^= (uint8_t)(1u << bit);
        }
    }
}

TP_TEST_CASE(crc, crc8_atm_is_deterministic_and_order_sensitive)
{
    const uint8_t a[] = { 0x01u, 0x02u, 0x03u };
    const uint8_t b[] = { 0x03u, 0x02u, 0x01u };
    uint8_t ca = tp_crc8_atm(a, sizeof(a));
    uint8_t cb = tp_crc8_atm(b, sizeof(b));

    TP_ASSERT_EQ((unsigned)ca, (unsigned)tp_crc8_atm(a, sizeof(a)));
    TP_ASSERT_NE((unsigned)ca, (unsigned)cb);
}

/* =========================================================================
 * Frame encoding
 * ========================================================================= */

TP_TEST_CASE(frame, encode_layout_is_exact)
{
    uint8_t buf[32];
    const uint8_t payload[3] = { 0x11u, 0x22u, 0x33u };
    uint32_t len = tp_frame_encode(buf, sizeof(buf), 0x10u, payload, 3u);

    TP_ASSERT_EQ(len, 9u);
    TP_ASSERT_EQ(buf[0], 0xAAu);
    TP_ASSERT_EQ(buf[1], 0x55u);
    TP_ASSERT_EQ(buf[2], 0x10u);
    TP_ASSERT_EQ(buf[3], 0x03u);
    TP_ASSERT_EQ(buf[4], 0x11u);
    TP_ASSERT_EQ(buf[5], 0x22u);
    TP_ASSERT_EQ(buf[6], 0x33u);
    /* CRC covers CMD | LEN | PAYLOAD, low byte first. */
    {
        uint16_t crc = tp_crc16_modbus(&buf[2], 5u);
        TP_ASSERT_EQ(buf[7], (uint8_t)(crc & 0xFFu));
        TP_ASSERT_EQ(buf[8], (uint8_t)((crc >> 8) & 0xFFu));
    }
}

TP_TEST_CASE(frame, encode_rejects_oversize_and_small_buffer)
{
    uint8_t buf[8];
    uint8_t payload[TP_PROTO_MAX_PAYLOAD + 4u];

    (void)memset(payload, 0, sizeof(payload));
    /* A payload beyond the protocol limit must be refused, not truncated: a
     * truncated frame with a valid CRC over the wrong data is the worst
     * possible outcome. */
    TP_ASSERT_EQ(tp_frame_encode(buf, sizeof(buf), 0x10u, payload,
                                 (uint8_t)(TP_PROTO_MAX_PAYLOAD + 1u)), 0u);
    TP_ASSERT_EQ(tp_frame_encode(buf, 4u, 0x10u, payload, 3u), 0u);
    TP_ASSERT_EQ(tp_frame_encode(NULL, sizeof(buf), 0x10u, payload, 3u), 0u);
}

TP_TEST_CASE(frame, round_trip_through_the_parser)
{
    tp_frame_parser_t p;
    tp_frame_t out;
    uint8_t buf[32];
    const uint8_t payload[4] = { 0xDEu, 0xADu, 0xBEu, 0xEFu };
    uint32_t len;
    uint32_t i;
    tp_status_t st = TP_ERR_NOT_READY;

    len = tp_frame_encode(buf, sizeof(buf), 0x21u, payload, 4u);
    tp_frame_parser_init(&p);
    for (i = 0u; i < len; ++i) {
        st = tp_frame_parser_feed(&p, buf[i], 1234u, &out);
    }
    TP_ASSERT_EQ(st, TP_OK);
    TP_ASSERT_EQ(out.cmd, 0x21u);
    TP_ASSERT_EQ(out.len, 4u);
    TP_ASSERT_EQ(memcmp(out.payload, payload, 4u), 0);
    TP_ASSERT_EQ(p.frames_ok, 1u);
    TP_ASSERT_EQ(p.last_frame_ms, 1234u);
}

TP_TEST_CASE(frame, decode_verifies_and_rejects)
{
    tp_frame_t out;
    uint8_t buf[32];
    const uint8_t payload[2] = { 0x01u, 0x02u };
    uint32_t len = tp_frame_encode(buf, sizeof(buf), 0x11u, payload, 2u);

    TP_ASSERT_EQ(tp_frame_decode(buf, len, &out), TP_OK);
    TP_ASSERT_EQ(out.cmd, 0x11u);

    buf[len - 1u] ^= 0xFFu; /* corrupt the CRC */
    TP_ASSERT_EQ(tp_frame_decode(buf, len, &out), TP_ERR_CRC);

    buf[len - 1u] ^= 0xFFu;   /* restore */
    buf[0] = 0x00u;           /* corrupt the header */
    TP_ASSERT_EQ(tp_frame_decode(buf, len, &out), TP_ERR_PROTOCOL);

    TP_ASSERT_EQ(tp_frame_decode(NULL, len, &out), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_frame_decode(buf, len, NULL), TP_ERR_PARAM);
    TP_ASSERT_EQ(tp_frame_decode(buf, 3u, &out), TP_ERR_PARAM);
}

/* =========================================================================
 * Parser resilience — the part that matters on a real link
 * ========================================================================= */

TP_TEST_CASE(frame, resynchronises_after_garbage)
{
    tp_frame_parser_t p;
    tp_frame_t out;
    uint8_t buf[32];
    const uint8_t garbage[5] = { 0x00u, 0xFFu, 0xAAu, 0x00u, 0x7Eu };
    uint32_t len;
    uint32_t i;
    tp_status_t st = TP_ERR_NOT_READY;

    len = tp_frame_encode(buf, sizeof(buf), 0x12u, NULL, 0u);
    tp_frame_parser_init(&p);
    for (i = 0u; i < sizeof(garbage); ++i) {
        (void)tp_frame_parser_feed(&p, garbage[i], 0u, &out);
    }
    for (i = 0u; i < len; ++i) {
        st = tp_frame_parser_feed(&p, buf[i], 0u, &out);
    }
    TP_ASSERT_EQ(st, TP_OK);
    TP_ASSERT_EQ(out.cmd, 0x12u);
    TP_ASSERT_GT(p.bytes_discarded, 0u);
}

TP_TEST_CASE(frame, resynchronises_on_a_doubled_header)
{
    tp_frame_parser_t p;
    tp_frame_t out;
    uint8_t buf[32];
    uint32_t len;
    uint32_t i;
    tp_status_t st = TP_ERR_NOT_READY;

    len = tp_frame_encode(buf, sizeof(buf), 0x13u, NULL, 0u);
    tp_frame_parser_init(&p);
    /* "AA AA 55 ..." - the second AA is the start of the real header.  A parser
     * that fell back to WAIT_HDR0 here would drop this frame. */
    (void)tp_frame_parser_feed(&p, 0xAAu, 0u, &out);
    for (i = 0u; i < len; ++i) {
        st = tp_frame_parser_feed(&p, buf[i], 0u, &out);
    }
    TP_ASSERT_EQ(st, TP_OK);
    TP_ASSERT_EQ(out.cmd, 0x13u);
}

TP_TEST_CASE(frame, rejects_an_oversize_length_immediately)
{
    tp_frame_parser_t p;
    tp_frame_t out;

    tp_frame_parser_init(&p);
    (void)tp_frame_parser_feed(&p, 0xAAu, 0u, &out);
    (void)tp_frame_parser_feed(&p, 0x55u, 0u, &out);
    (void)tp_frame_parser_feed(&p, 0x10u, 0u, &out);
    /* A length beyond the limit must be rejected at once.  Waiting for the
     * bytes would stall the parser indefinitely and swallow the next real
     * frame as payload. */
    TP_ASSERT_EQ(tp_frame_parser_feed(&p, (uint8_t)(TP_PROTO_MAX_PAYLOAD + 1u),
                                      0u, &out),
                 TP_ERR_OVERFLOW);
    TP_ASSERT_EQ(p.length_errors, 1u);
    TP_ASSERT_EQ(tp_frame_parser_state(&p), TP_FRAME_WAIT_HDR0);
}

TP_TEST_CASE(frame, a_corrupt_crc_does_not_swallow_the_next_frame)
{
    tp_frame_parser_t p;
    tp_frame_t out;
    uint8_t bad[32];
    uint8_t good[32];
    const uint8_t payload[2] = { 0xAAu, 0x55u }; /* header-like payload */
    uint32_t bad_len;
    uint32_t good_len;
    uint32_t i;
    uint32_t ok = 0u;

    bad_len = tp_frame_encode(bad, sizeof(bad), 0x20u, payload, 2u);
    bad[bad_len - 1u] ^= 0x5Au; /* break the CRC */
    good_len = tp_frame_encode(good, sizeof(good), 0x21u, NULL, 0u);

    tp_frame_parser_init(&p);
    for (i = 0u; i < bad_len; ++i) {
        (void)tp_frame_parser_feed(&p, bad[i], 0u, &out);
    }
    for (i = 0u; i < good_len; ++i) {
        if (tp_frame_parser_feed(&p, good[i], 0u, &out) == TP_OK) {
            ok++;
        }
    }
    /* The payload deliberately contains "AA 55".  If a CRC failure consumed the
     * whole candidate frame, the parser would resynchronise inside the payload
     * and either lose the next frame or emit garbage. */
    TP_ASSERT_EQ(ok, 1u);
    TP_ASSERT_EQ(out.cmd, 0x21u);
    TP_ASSERT_EQ(p.crc_errors, 1u);
}

TP_TEST_CASE(frame, a_zero_length_frame_round_trips)
{
    tp_frame_parser_t p;
    tp_frame_t out;
    uint8_t buf[16];
    uint32_t len = tp_frame_encode(buf, sizeof(buf), 0x01u, NULL, 0u);
    uint32_t i;
    tp_status_t st = TP_ERR_NOT_READY;

    TP_ASSERT_EQ(len, 6u);
    tp_frame_parser_init(&p);
    for (i = 0u; i < len; ++i) {
        st = tp_frame_parser_feed(&p, buf[i], 0u, &out);
    }
    TP_ASSERT_EQ(st, TP_OK);
    TP_ASSERT_EQ(out.len, 0u);
}

TP_TEST_CASE(frame, maximum_length_frame_round_trips)
{
    tp_frame_parser_t p;
    tp_frame_t out;
    uint8_t buf[TP_PROTO_MAX_FRAME + 8u];
    uint8_t payload[TP_PROTO_MAX_PAYLOAD];
    uint32_t len;
    uint32_t i;
    tp_status_t st = TP_ERR_NOT_READY;

    for (i = 0u; i < sizeof(payload); ++i) {
        payload[i] = (uint8_t)i;
    }
    len = tp_frame_encode(buf, sizeof(buf), 0xC0u, payload,
                          (uint8_t)TP_PROTO_MAX_PAYLOAD);
    TP_ASSERT_EQ(len, TP_PROTO_MAX_PAYLOAD + 6u);

    tp_frame_parser_init(&p);
    for (i = 0u; i < len; ++i) {
        st = tp_frame_parser_feed(&p, buf[i], 0u, &out);
    }
    TP_ASSERT_EQ(st, TP_OK);
    TP_ASSERT_EQ(out.len, (uint8_t)TP_PROTO_MAX_PAYLOAD);
    TP_ASSERT_EQ(memcmp(out.payload, payload, sizeof(payload)), 0);
}

TP_TEST_CASE(frame, stray_payload_tail_is_zeroed)
{
    tp_frame_parser_t p;
    tp_frame_t out;
    uint8_t buf[32];
    const uint8_t long_payload[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    const uint8_t short_payload[2] = { 0xEEu, 0xFFu };
    uint32_t len;
    uint32_t i;
    tp_status_t st = TP_ERR_NOT_READY;

    len = tp_frame_encode(buf, sizeof(buf), 0x20u, long_payload, 10u);
    tp_frame_parser_init(&p);
    for (i = 0u; i < len; ++i) {
        st = tp_frame_parser_feed(&p, buf[i], 0u, &out);
    }
    TP_ASSERT_EQ(st, TP_OK);

    len = tp_frame_encode(buf, sizeof(buf), 0x20u, short_payload, 2u);
    for (i = 0u; i < len; ++i) {
        st = tp_frame_parser_feed(&p, buf[i], 0u, &out);
    }
    TP_ASSERT_EQ(st, TP_OK);
    /* Bytes past `len` must not still hold the previous frame's data: a decoder
     * that reads past the length would otherwise silently see stale values. */
    TP_ASSERT_EQ(out.payload[2], 0u);
    TP_ASSERT_EQ(out.payload[9], 0u);
}
