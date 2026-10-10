/* Host test for rover_link.h. Runs on a PC, no hardware.
 *
 *   gcc -O2 -std=c99 -Wall -I esp32cam/LinkTest test/test_link.c -o test_link && ./test_link
 *
 * Checks: the CRC check values, COBS round trips at every length (including runs of 254+
 * non-zero bytes, the one case COBS treats specially), and a long stream of real packets with
 * junk spliced in and bits flipped, where the receiver must recover every packet that was not
 * touched and count exactly the ones that were.
 */
#include <stdio.h>
#include <stdlib.h>
#include "rover_link.h"

static int fails;
#define CHECK(c, ...)                                                                                       \
    do {                                                                                                    \
        if (!(c)) {                                                                                         \
            fails++;                                                                                        \
            printf("FAIL line %d: ", __LINE__);                                                             \
            printf(__VA_ARGS__);                                                                            \
            printf("\n");                                                                                   \
        }                                                                                                   \
    } while (0)

static uint32_t rng = 12345;
static uint32_t rnd(void)
{
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return rng;
}

static void test_crc(void)
{
    const uint8_t s[] = "123456789";
    CHECK(link_crc16(s, 9) == 0x29B1, "crc16 = %04x", link_crc16(s, 9));
    CHECK(link_crc32(0, s, 9) == 0xCBF43926u, "crc32 = %08x", (unsigned)link_crc32(0, s, 9));
    CHECK(link_crc32(link_crc32(0, s, 4), s + 4, 5) == 0xCBF43926u, "crc32 not chainable");
}

static void test_cobs(void)
{
    static uint8_t src[1200], enc[1300], dec[1200];
    for (int kind = 0; kind < 4; kind++)
        for (size_t n = 0; n <= 1100; n++) {
            for (size_t i = 0; i < n; i++) {
                uint32_t r = rnd();
                src[i] = kind == 0 ? (uint8_t)r                    /* random                   */
                       : kind == 1 ? (uint8_t)(1 + r % 255)        /* no zeros: 254-byte runs  */
                       : kind == 2 ? 0                             /* all zeros                */
                                   : (uint8_t)((r % 3) ? 0 : r);   /* mostly zeros             */
            }
            size_t m = link_cobs_encode(src, n, enc);
            int zero = 0;
            for (size_t i = 0; i < m; i++) zero |= (enc[i] == 0);
            CHECK(!zero, "kind %d n %u: encoded data contains 0x00", kind, (unsigned)n);
            CHECK(m <= n + n / 254 + 1, "kind %d n %u: encoded %u bytes, over the bound", kind, (unsigned)n,
                  (unsigned)m);
            int d = link_cobs_decode(enc, m, dec, sizeof dec);
            CHECK(d == (int)n && memcmp(src, dec, n) == 0, "kind %d n %u: round trip gave %d", kind,
                  (unsigned)n, d);
        }
}

static size_t make_test(uint16_t seq, uint32_t counter, uint8_t *wire)
{
    uint8_t p[LINK_MAX_PAYLOAD];
    size_t n = link_test_len(counter);
    link_put_u32(p, counter);
    for (size_t i = 4; i < n; i++) p[i] = link_test_byte(counter, (uint32_t)i);
    return link_pack(LINK_TEST, seq, p, n, wire);
}

/* Every legal length, back to back, then the same with damage. */
static void test_stream(void)
{
    static uint8_t stream[4000000];
    static uint8_t wire[LINK_WIRE_MAX];
    size_t len = 0;
    const uint32_t N = 20000;
    uint32_t junk_runs = 0, flipped = 0;
    static uint8_t damaged[20000];

    /* junk before the first packet, as the ESP32's IO14 produces at boot, ended by its 0x00 */
    for (int i = 0; i < 37; i++) stream[len++] = (uint8_t)(rnd() | 1);
    stream[len++] = 0;

    for (uint32_t k = 0; k < N; k++) {
        size_t m = make_test((uint16_t)k, k, wire);
        CHECK(m <= LINK_WIRE_MAX, "packet %u is %u bytes, over LINK_WIRE_MAX", (unsigned)k, (unsigned)m);
        if (k % 97 == 50) { /* flip one bit inside the packet, never in the delimiter, and never
                               into a 0x00 (that would split it in two: a different test) */
            size_t at;
            uint8_t bit;
            do {
                at = rnd() % (m - 1);
                bit = (uint8_t)(1u << (rnd() % 8));
            } while ((wire[at] ^ bit) == 0);
            wire[at] ^= bit;
            damaged[k] = 1;
            flipped++;
        }
        memcpy(stream + len, wire, m);
        len += m;
        if (k % 501 == 200) { /* junk between packets */
            int j = 1 + rnd() % 40;
            for (int i = 0; i < j; i++) stream[len++] = (uint8_t)(rnd() | 1);
            stream[len++] = 0;
            junk_runs++;
        }
    }

    link_rx_t r;
    memset(&r, 0, sizeof r);
    uint32_t good = 0, pattern_bad = 0, wrong_seq = 0, expect_next = 0;
    for (size_t i = 0; i < len; i++) {
        if (!link_rx_byte(&r, stream[i])) continue;
        CHECK(r.type == LINK_TEST, "type %u", r.type);
        uint32_t c = link_get_u32(r.payload);
        int bad = (r.len != link_test_len(c));
        for (size_t j = 4; !bad && j < r.len; j++) bad = (r.payload[j] != link_test_byte(c, (uint32_t)j));
        pattern_bad += bad;
        while (expect_next < N && damaged[expect_next]) expect_next++;
        wrong_seq += (r.seq != (uint16_t)expect_next || c != expect_next);
        expect_next++;
        good++;
    }
    printf("stream: %u packets, %u bytes, %u flipped, %u junk runs -> ok %u, crc %u, cobs %u, long %u, lost %u\n",
           (unsigned)N, (unsigned)len, (unsigned)flipped, (unsigned)junk_runs, (unsigned)r.ok,
           (unsigned)r.crc_err, (unsigned)r.cobs_err, (unsigned)r.too_long, (unsigned)r.lost);
    CHECK(good == N - flipped, "recovered %u of %u undamaged packets", (unsigned)good, (unsigned)(N - flipped));
    CHECK(pattern_bad == 0, "%u packets passed the CRC with wrong content", (unsigned)pattern_bad);
    CHECK(wrong_seq == 0, "%u packets out of order", (unsigned)wrong_seq);
    CHECK(r.lost == flipped, "lost %u, expected %u", (unsigned)r.lost, (unsigned)flipped);
    CHECK(link_rx_bad(&r) == flipped + junk_runs + 1, "bad %u, expected %u", (unsigned)link_rx_bad(&r),
          (unsigned)(flipped + junk_runs + 1));
}

/* A sender reboot: seq jumps back to 0. Must count a restart, not 60000 lost packets. */
static void test_restart(void)
{
    uint8_t wire[LINK_WIRE_MAX];
    link_rx_t r;
    memset(&r, 0, sizeof r);
    for (uint16_t s = 5000; s < 5010; s++) {
        size_t m = link_pack(LINK_HEARTBEAT, s, NULL, 0, wire);
        for (size_t i = 0; i < m; i++) link_rx_byte(&r, wire[i]);
    }
    for (uint16_t s = 0; s < 3; s++) {
        size_t m = link_pack(LINK_HEARTBEAT, s, NULL, 0, wire);
        for (size_t i = 0; i < m; i++) link_rx_byte(&r, wire[i]);
    }
    CHECK(r.ok == 13 && r.lost == 0 && r.restarts == 1, "ok %u lost %u restarts %u", (unsigned)r.ok,
          (unsigned)r.lost, (unsigned)r.restarts);

    /* and seq wrapping 65535 -> 0 is NOT a restart */
    memset(&r, 0, sizeof r);
    for (uint32_t s = 65530; s < 65540; s++) {
        size_t m = link_pack(LINK_HEARTBEAT, (uint16_t)s, NULL, 0, wire);
        for (size_t i = 0; i < m; i++) link_rx_byte(&r, wire[i]);
    }
    CHECK(r.ok == 10 && r.lost == 0 && r.restarts == 0, "wrap: ok %u lost %u restarts %u", (unsigned)r.ok,
          (unsigned)r.lost, (unsigned)r.restarts);
}

/* A packet that never ends (lost 0x00 / wrong baud) is dropped as too long, and the next one
 * still gets through. */
static void test_too_long(void)
{
    uint8_t wire[LINK_WIRE_MAX];
    link_rx_t r;
    memset(&r, 0, sizeof r);
    for (int i = 0; i < 1000; i++) link_rx_byte(&r, 0x55);
    link_rx_byte(&r, 0);
    size_t m = make_test(7, 7, wire);
    for (size_t i = 0; i < m; i++) link_rx_byte(&r, wire[i]);
    CHECK(r.too_long == 1 && r.ok == 1, "too_long %u ok %u", (unsigned)r.too_long, (unsigned)r.ok);
}

/* --dump FILE: write a fixed packet stream for `python tools/link_tool.py selftest FILE`, which
 * re-encodes the same packets in Python and must produce the identical bytes. */
static int dump(const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) return 1;
    uint8_t wire[LINK_WIRE_MAX], hb[LINK_HB_LEN];
    for (uint32_t k = 0; k < 600; k++) fwrite(wire, 1, make_test((uint16_t)k, k, wire), f);
    fwrite(wire, 1, link_pack(LINK_LOG, 600, (const uint8_t *)"hello from C", 12, wire), f);
    link_put_u32(hb + 0, 1);
    link_put_u32(hb + 4, 2);
    link_put_u32(hb + 8, 3);
    link_put_u32(hb + 12, 4);
    hb[16] = MODE_CAMERA;
    fwrite(wire, 1, link_pack(LINK_HEARTBEAT, 601, hb, sizeof hb, wire), f);
    fclose(f);
    printf("wrote %s\n", path);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc == 3 && strcmp(argv[1], "--dump") == 0) return dump(argv[2]);
    test_crc();
    test_cobs();
    test_stream();
    test_restart();
    test_too_long();
    printf(fails ? "%d FAILED\n" : "all link tests passed\n", fails);
    return fails != 0;
}
