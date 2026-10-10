/* rover_link.h: the wire protocol between the ESP32-CAM and the MCX N947.
 *
 * This is the link from the build plan, Level 4 ("The link between MCU and ESP32"): UART,
 * COBS-framed packets, CRC-16. This file is header-only and compiles unchanged as C99 (the
 * MCX N947 firmware) and as C++ (the Arduino sketch). THIS copy, next to LinkTest.ino, is the
 * only one to edit. nxp/install.py copies it into the SDK example, and tools/link_tool.py is
 * a line-by-line Python twin of it.
 *
 * One packet on the wire:
 *
 *     COBS( type:u8 | seq:u16le | payload: 0..LINK_MAX_PAYLOAD bytes | crc:u16le )  0x00
 *
 * COBS (Consistent Overhead Byte Stuffing) rewrites the packet so it contains no 0x00 byte.
 * It costs one extra byte per 254. After that, a 0x00 on the wire always means "a packet
 * ends here". A receiver that powers up in the middle of a packet, or loses a byte, throws away
 * only that one packet and is back in step at the next 0x00.
 *
 * crc is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final xor) over
 * type, seq and payload. It catches the bit errors that COBS alone would pass.
 *
 * seq goes up by one for every packet a side sends, so the receiver can count the packets that
 * never arrived intact. It wraps at 65535. A big backwards jump means the sender rebooted.
 *
 * All multi-byte fields are little endian and are written with link_put_* / link_get_*, never
 * by casting a struct. The ESP32 and the Cortex-M33 are both little endian, but the helpers keep
 * the byte layout independent of compiler padding.
 */
#ifndef ROVER_LINK_H
#define ROVER_LINK_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define LINK_MAX_PAYLOAD 240u
#define LINK_RAW_MAX     (3u + LINK_MAX_PAYLOAD + 2u)            /* type + seq + payload + crc  */
#define LINK_WIRE_MAX    (LINK_RAW_MAX + LINK_RAW_MAX / 254u + 2u) /* COBS overhead + the 0x00 */

/* Packet types. The first five are the rover's real message set from the build plan. The rest
 * exist for bring-up: this test uses them, the rover will not. */
enum {
    LINK_TELEMETRY    = 0x01, /* MCU -> ESP32, 20 Hz: pose, speeds, estimator outputs, battery  */
    LINK_EVENT        = 0x02, /* MCU -> ESP32: stuck, escaped, collision, lifted, low battery   */
    LINK_SKILL_CMD    = 0x03, /* ESP32 -> MCU                                                    */
    LINK_SKILL_STATUS = 0x04, /* MCU -> ESP32                                                    */
    LINK_HEARTBEAT    = 0x05, /* both ways, 5 Hz. Silence for 1 s means the other side is gone  */

    LINK_TEST         = 0x10, /* ESP32 -> MCU: u32 counter, then bytes of known content          */
    LINK_FRAME_START  = 0x11, /* ESP32 -> MCU: a camera frame follows (layout below)              */
    LINK_FRAME_CHUNK  = 0x12, /* ESP32 -> MCU: u16 frame id, u32 offset, then frame bytes        */
    LINK_MODE         = 0x13, /* MCU -> ESP32: u8 mode, what the ESP32 should send               */
    LINK_LOG          = 0x14  /* ESP32 -> MCU: one line of text for the MCU console              */
};

/* What the ESP32 sends, chosen by a LINK_MODE packet from the MCU. */
enum {
    MODE_IDLE   = 0, /* heartbeats only                                                         */
    MODE_TEST   = 1, /* LINK_TEST packets at 50 per second                                       */
    MODE_FLOOD  = 2, /* LINK_TEST packets back to back, as fast as the UART goes                */
    MODE_CAMERA = 3  /* camera frames, 160x120 8-bit grayscale, sent in chunks                  */
};

/* LINK_HEARTBEAT payload: what the sender has received FROM THE OTHER SIDE. Both directions
 * use the same layout, so each side can see how its own packets are arriving.
 *   u32 uptime_ms | u32 rx_ok | u32 rx_bad | u32 rx_lost | u8 mode                 = 17 bytes */
#define LINK_HB_LEN 17u

/* LINK_FRAME_START payload:
 *   u16 frame_id | u16 width | u16 height | u8 format | u32 total_len | u32 crc32  = 15 bytes
 * format 0 = 8-bit grayscale, row by row. crc32 is CRC-32/ISO-HDLC (the zlib one) of the
 * whole image, so the MCU can prove the reassembled frame is byte-for-byte what was sent. */
#define LINK_FS_LEN      15u
#define LINK_FMT_GRAY8   0u
#define LINK_CHUNK_HDR   6u                                  /* u16 frame id + u32 offset */
#define LINK_CHUNK_DATA  (LINK_MAX_PAYLOAD - LINK_CHUNK_HDR) /* image bytes per chunk     */

/* ----------------------------------------------------------------- little-endian helpers */
static inline void link_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
static inline void link_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static inline uint16_t link_get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}
static inline uint32_t link_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ----------------------------------------------------------------------------- checksums */
/* CRC-16/CCITT-FALSE. Check value: "123456789" -> 0x29B1. */
static inline uint16_t link_crc16(const uint8_t *p, size_t n)
{
    uint16_t c = 0xFFFFu;
    while (n--) {
        c ^= (uint16_t)(*p++ << 8);
        for (int b = 0; b < 8; b++)
            c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u) : (uint16_t)(c << 1);
    }
    return c;
}

/* CRC-32/ISO-HDLC, the zlib / Ethernet one, chainable: start from 0 and feed the data in any
 * number of pieces, link_crc32(link_crc32(0, a), b) == crc of a followed by b.
 * Check value: "123456789" -> 0xCBF43926. */
static inline uint32_t link_crc32(uint32_t c, const uint8_t *p, size_t n)
{
    c = ~c;
    while (n--) {
        c ^= *p++;
        for (int b = 0; b < 8; b++)
            c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

/* ---------------------------------------------------------------------------------- COBS */
/* Encode n bytes into dst, which needs n + n / 254 + 1 bytes. The 0x00 delimiter is NOT
 * written. Returns the encoded length.
 *
 * Each block starts with a code byte c: copy the next c - 1 bytes, then (if c < 0xFF) a 0x00
 * that was removed. code_at remembers where the current block's code byte will go. */
static inline size_t link_cobs_encode(const uint8_t *src, size_t n, uint8_t *dst)
{
    size_t code_at = 0, o = 1;
    uint8_t code = 1;
    for (size_t i = 0; i < n; i++) {
        if (src[i] == 0) {
            dst[code_at] = code;
            code_at = o++;
            code = 1;
        } else {
            dst[o++] = src[i];
            if (++code == 0xFF) { /* 254 bytes without a zero: close the block, no zero implied */
                dst[code_at] = code;
                code_at = o++;
                code = 1;
            }
        }
    }
    dst[code_at] = code;
    return o;
}

/* Decode one packet (delimiter already removed) into dst, room for max bytes.
 * Returns the decoded length, or -1 if it is not valid COBS or does not fit. */
static inline int link_cobs_decode(const uint8_t *src, size_t n, uint8_t *dst, size_t max)
{
    size_t i = 0, o = 0;
    while (i < n) {
        uint8_t code = src[i++];
        if (code == 0)
            return -1;
        for (uint8_t k = 1; k < code; k++) {
            if (i >= n || o >= max)
                return -1; /* block runs past the end of the packet */
            dst[o++] = src[i++];
        }
        if (code != 0xFF && i < n) {
            if (o >= max)
                return -1;
            dst[o++] = 0;
        }
    }
    return (int)o;
}

/* --------------------------------------------------------------------------------- send */
/* Build one complete wire packet in out (LINK_WIRE_MAX bytes): COBS-encoded and ending in
 * 0x00. Payloads longer than LINK_MAX_PAYLOAD are cut. Returns the number of bytes to send. */
static inline size_t link_pack(uint8_t type, uint16_t seq, const uint8_t *payload, size_t n,
                               uint8_t *out)
{
    uint8_t raw[LINK_RAW_MAX];
    if (n > LINK_MAX_PAYLOAD)
        n = LINK_MAX_PAYLOAD;
    raw[0] = type;
    link_put_u16(raw + 1, seq);
    if (n)
        memcpy(raw + 3, payload, n);
    link_put_u16(raw + 3 + n, link_crc16(raw, 3 + n));
    size_t m = link_cobs_encode(raw, 3 + n + 2, out);
    out[m++] = 0;
    return m;
}

/* ------------------------------------------------------------------------------ receive */
/* Receiver state. Zero it (memset or = {0}) before use, then call link_rx_byte for every byte
 * that arrives. All counters only ever go up. */
typedef struct {
    uint8_t enc[LINK_WIRE_MAX]; /* encoded bytes of the packet being received                 */
    size_t n;
    int overflow;               /* this packet grew past LINK_WIRE_MAX: drop it at its 0x00 */
    uint8_t raw[LINK_RAW_MAX];  /* the last good packet, decoded                              */

    /* valid after link_rx_byte returns 1, until the next call */
    uint8_t type;
    uint16_t seq;
    const uint8_t *payload;
    size_t len;

    uint32_t bytes;    /* every byte seen, delimiters and garbage included                  */
    uint32_t ok;       /* packets that decoded and passed the CRC                            */
    uint32_t crc_err;  /* decoded, but the CRC did not match: bits flipped on the wire       */
    uint32_t cobs_err; /* not even valid COBS, or shorter than a header + CRC                */
    uint32_t too_long; /* longer than any legal packet: a lost 0x00 or a baud mismatch       */
    uint32_t lost;     /* seq numbers that never arrived intact (corrupted ones included)   */
    uint32_t restarts; /* seq jumped backwards: the sender rebooted                          */
    int have_seq;
    uint16_t next_seq;
} link_rx_t;

static inline uint32_t link_rx_bad(const link_rx_t *r)
{
    return r->crc_err + r->cobs_err + r->too_long;
}

/* Feed one received byte. Returns 1 when that byte completes a good packet; then type, seq,
 * payload and len describe it. Returns 0 otherwise. */
static inline int link_rx_byte(link_rx_t *r, uint8_t b)
{
    r->bytes++;
    if (b != 0) {
        if (r->n < sizeof r->enc)
            r->enc[r->n++] = b;
        else
            r->overflow = 1;
        return 0;
    }
    if (r->n == 0 && !r->overflow)
        return 0; /* two 0x00 in a row: an empty packet, used to flush, not an error */

    size_t n = r->n;
    int overflow = r->overflow;
    r->n = 0;
    r->overflow = 0;
    if (overflow) {
        r->too_long++;
        return 0;
    }
    int m = link_cobs_decode(r->enc, n, r->raw, sizeof r->raw);
    if (m < 5) { /* type + seq + crc is the smallest packet */
        r->cobs_err++;
        return 0;
    }
    if (link_crc16(r->raw, (size_t)m - 2) != link_get_u16(r->raw + m - 2)) {
        r->crc_err++;
        return 0;
    }
    r->type = r->raw[0];
    r->seq = link_get_u16(r->raw + 1);
    r->payload = r->raw + 3;
    r->len = (size_t)m - 5;

    if (r->have_seq) {
        uint16_t gap = (uint16_t)(r->seq - r->next_seq);
        if (gap < 2000u)
            r->lost += gap;
        else
            r->restarts++; /* a gap this big is a reboot (seq back near 0), not 2000 lost packets */
    }
    r->have_seq = 1;
    r->next_seq = (uint16_t)(r->seq + 1);
    r->ok++;
    return 1;
}

/* ----------------------------------------------------------------------- test patterns */
/* Byte i of LINK_TEST packet number `counter`. Every fifth byte is 0x00 so that COBS is
 * exercised on every packet. The receiver recomputes this and compares, which catches a bug
 * that the CRC cannot: the same mistake made on both sides. */
static inline uint8_t link_test_byte(uint32_t counter, uint32_t i)
{
    return ((counter + i) % 5u == 0u) ? 0u : (uint8_t)(counter * 31u + i * 7u);
}

/* Payload length of LINK_TEST packet number `counter`: walks through every legal length,
 * 4 (the counter alone) up to LINK_MAX_PAYLOAD, so every size gets tested. */
static inline size_t link_test_len(uint32_t counter)
{
    return 4u + (size_t)(counter % (LINK_MAX_PAYLOAD - 4u + 1u));
}

#endif /* ROVER_LINK_H */
