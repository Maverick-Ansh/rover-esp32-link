/* ESP32-CAM -> MCX N947 link test (rover build plan, Level 4: "The link between MCU and ESP32").
 *
 * The ESP32-CAM sends packets over LPUART0 (P0_12 / P0_13, header J8) in the rover protocol of
 * rover_link.h: COBS framing, CRC-16, a sequence number per packet. This firmware receives them,
 * checks every byte, and prints once a second on the MCU-Link VCOM (115200 8N1) how many arrived,
 * how many were damaged or missing, and how fast they came. It also sends its own heartbeat back
 * 5 times a second, carrying those counts, so the ESP32's Serial Monitor shows the same numbers:
 * that proves the MCU -> ESP32 direction too.
 *
 * How bytes get in: the LPUART0 receive interrupt copies every byte into a 16 KB ring buffer and
 * returns. main() empties the ring and feeds link_rx_byte(), which rebuilds packets. The
 * interrupt never parses anything, so it stays short even at 921600 baud (one byte every 10.8 us).
 *
 * Console keys (type in any serial terminal on the MCU-Link VCOM; Enter is ignored):
 *   t   ESP32 -> test packets, 50 per second, every length 4..240 bytes  (the ESP32's default)
 *   f   ESP32 -> flood: test packets back to back, measures real throughput
 *   c   ESP32 -> camera: 160x120 grayscale frames, checked with CRC-32 after reassembly
 *   x   ESP32 -> idle: heartbeats only
 *   i   print the last good camera frame as ASCII art (cover the lens and look again)
 *   r   reset all counters
 *   h   help
 */
#include <string.h>
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "board.h"
#include "app.h"
#include "rover_link.h"

#define LED_R_PIN 10U /* GPIO0, active low */
#define LED_G_PIN 27U /* GPIO0, active low */
#define LED_B_PIN 2U  /* GPIO1, active low */

#define HEARTBEAT_MS 200U  /* 5 Hz, as in the build plan */
#define PEER_LOST_MS 1000U /* no heartbeat for 1 s: the ESP32 is gone */

#define RING_SIZE 16384U   /* power of two. 16 KB holds 0.18 s of traffic at 921600 baud */
#define FRAME_MAX 32768U   /* biggest camera frame accepted (160x120 gray is 19200)    */

/* ------------------------------------------------------------------------- time base */
static volatile uint32_t g_ms;

void SysTick_Handler(void)
{
    g_ms++;
}

/* ------------------------------------------------------------- receive interrupt + ring */
static volatile uint8_t g_ring[RING_SIZE];
static volatile uint32_t g_head, g_tail;  /* the ISR moves head, main() moves tail */
static volatile uint32_t g_ring_drop;     /* bytes lost because main() fell behind     */
static volatile uint32_t g_overrun;       /* bytes lost because the ISR fell behind    */
static volatile uint32_t g_framing;       /* stop bit was 0: wrong baud, or a bad wire */
static volatile uint32_t g_noise;         /* the line wobbled inside a bit: noise      */

#define UART_ERRS (kLPUART_RxOverrunFlag | kLPUART_FramingErrorFlag | kLPUART_NoiseErrorFlag)

void LINK_IRQ_HANDLER(void)
{
    uint32_t st = LPUART_GetStatusFlags(LINK_UART);
    if (st & UART_ERRS) {
        if (st & kLPUART_RxOverrunFlag) g_overrun++; /* receive stops until this is cleared */
        if (st & kLPUART_FramingErrorFlag) g_framing++;
        if (st & kLPUART_NoiseErrorFlag) g_noise++;
        (void)LPUART_ClearStatusFlags(LINK_UART, st & UART_ERRS);
    }
    while (LPUART_GetRxFifoCount(LINK_UART)) {
        uint8_t b = LPUART_ReadByte(LINK_UART);
        uint32_t h = g_head;
        if (h - g_tail < RING_SIZE) {
            g_ring[h & (RING_SIZE - 1U)] = b;
            g_head = h + 1U;
        } else {
            g_ring_drop++;
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

/* ------------------------------------------------------------------------- link state */
static link_rx_t g_rx;
static uint16_t g_seq;               /* seq of the next packet we send */
static uint8_t g_asked_mode = 0xFF;  /* last mode we asked for, 0xFF = none yet */

/* from the ESP32's heartbeat: what it says about itself and about our packets */
static uint8_t g_peer_mode = 0xFF;
static uint32_t g_peer_ok, g_peer_bad, g_peer_lost;
static uint32_t g_last_hb, g_hb_count;
static int g_peer_alive;

/* test packets */
static uint32_t g_test_ok, g_pattern_err;

/* camera frames */
static uint8_t g_img[FRAME_MAX];     /* the frame being reassembled */
static uint8_t g_last[FRAME_MAX];    /* the last frame that passed its CRC-32 */
static struct {
    int active;
    uint16_t id, w, h;
    uint32_t total, crc_expect, crc, got;
} g_fr;
static uint16_t g_last_w, g_last_h;
static uint32_t g_frames_ok, g_frames_crc, g_frames_incomplete, g_last_mean;

static const char *mode_name(uint8_t m)
{
    switch (m) {
        case MODE_IDLE: return "IDLE";
        case MODE_TEST: return "TEST";
        case MODE_FLOOD: return "FLOOD";
        case MODE_CAMERA: return "CAMERA";
        default: return "?";
    }
}

static void leds(int r, int g, int b)
{
    GPIO_PinWrite(GPIO0, LED_R_PIN, !r);
    GPIO_PinWrite(GPIO0, LED_G_PIN, !g);
    GPIO_PinWrite(GPIO1, LED_B_PIN, !b);
}

static void send(uint8_t type, const uint8_t *payload, size_t n)
{
    uint8_t wire[LINK_WIRE_MAX];
    size_t m = link_pack(type, g_seq++, payload, n, wire);
    (void)LPUART_WriteBlocking(LINK_UART, wire, m);
}

static void send_heartbeat(void)
{
    uint8_t p[LINK_HB_LEN];
    link_put_u32(p + 0, g_ms);
    link_put_u32(p + 4, g_rx.ok);
    link_put_u32(p + 8, link_rx_bad(&g_rx));
    link_put_u32(p + 12, g_rx.lost);
    p[16] = g_asked_mode;
    send(LINK_HEARTBEAT, p, sizeof p);
}

static void ask_mode(uint8_t m)
{
    g_asked_mode = m;
    send(LINK_MODE, &m, 1);
    PRINTF("asked the ESP32 for mode %s\r\n", mode_name(m));
}

/* ----------------------------------------------------------------------- packet handling */
static void on_test(const uint8_t *p, size_t n)
{
    if (n < 4U) {
        g_pattern_err++;
        return;
    }
    uint32_t counter = link_get_u32(p);
    int bad = (n != link_test_len(counter));
    for (size_t i = 4; !bad && i < n; i++)
        bad = (p[i] != link_test_byte(counter, (uint32_t)i));
    if (bad)
        g_pattern_err++;
    else
        g_test_ok++;
}

static void on_frame_start(const uint8_t *p, size_t n)
{
    if (n < LINK_FS_LEN) return;
    if (g_fr.active) g_frames_incomplete++; /* the previous frame never finished */
    g_fr.id = link_get_u16(p + 0);
    g_fr.w = link_get_u16(p + 2);
    g_fr.h = link_get_u16(p + 4);
    g_fr.total = link_get_u32(p + 7);
    g_fr.crc_expect = link_get_u32(p + 11);
    g_fr.crc = 0;
    g_fr.got = 0;
    g_fr.active = (p[6] == LINK_FMT_GRAY8 && g_fr.total <= FRAME_MAX &&
                   g_fr.total == (uint32_t)g_fr.w * g_fr.h);
    if (!g_fr.active) PRINTF("frame %u refused: %ux%u, %u bytes\r\n", g_fr.id, g_fr.w, g_fr.h, g_fr.total);
}

static void on_frame_chunk(const uint8_t *p, size_t n)
{
    if (!g_fr.active || n < LINK_CHUNK_HDR || link_get_u16(p) != g_fr.id) return;
    uint32_t off = link_get_u32(p + 2), len = (uint32_t)(n - LINK_CHUNK_HDR);
    if (off != g_fr.got || off + len > g_fr.total) { /* a chunk went missing in between */
        g_frames_incomplete++;
        g_fr.active = 0;
        return;
    }
    memcpy(g_img + off, p + LINK_CHUNK_HDR, len);
    g_fr.crc = link_crc32(g_fr.crc, p + LINK_CHUNK_HDR, len);
    g_fr.got += len;
    if (g_fr.got < g_fr.total) return;

    g_fr.active = 0;
    if (g_fr.crc != g_fr.crc_expect) {
        g_frames_crc++;
        return;
    }
    uint32_t sum = 0;
    for (uint32_t i = 0; i < g_fr.total; i++) sum += g_img[i];
    g_last_mean = sum / g_fr.total;
    memcpy(g_last, g_img, g_fr.total);
    g_last_w = g_fr.w;
    g_last_h = g_fr.h;
    g_frames_ok++;
}

static void on_packet(void)
{
    const uint8_t *p = g_rx.payload;
    size_t n = g_rx.len;
    switch (g_rx.type) {
        case LINK_HEARTBEAT:
            if (n >= LINK_HB_LEN) {
                g_peer_ok = link_get_u32(p + 4);
                g_peer_bad = link_get_u32(p + 8);
                g_peer_lost = link_get_u32(p + 12);
                g_peer_mode = p[16];
            }
            g_last_hb = g_ms;
            g_hb_count++;
            if (!g_peer_alive) PRINTF("ESP32 heartbeat: FOUND\r\n");
            g_peer_alive = 1;
            break;
        case LINK_TEST: on_test(p, n); break;
        case LINK_FRAME_START: on_frame_start(p, n); break;
        case LINK_FRAME_CHUNK: on_frame_chunk(p, n); break;
        case LINK_LOG: {
            char s[LINK_MAX_PAYLOAD + 1];
            memcpy(s, p, n);
            s[n] = 0;
            PRINTF("ESP32 says: %s\r\n", s);
            break;
        }
        default: break;
    }
}

/* Empty the ring into the packet decoder. Also called between lines of long printouts: PRINTF
 * is blocking at 115200, and while it runs the ESP32 keeps sending at up to 8x that rate. */
static void pump(void)
{
    while (g_tail != g_head) {
        uint8_t b = g_ring[g_tail & (RING_SIZE - 1U)];
        g_tail++;
        if (link_rx_byte(&g_rx, b)) on_packet();
    }
}

/* -------------------------------------------------------------------------- console */
static int console_getc(void)
{
    uint32_t st = LPUART_GetStatusFlags(LPUART4);
    if (st & kLPUART_RxOverrunFlag) (void)LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);
    return (st & kLPUART_RxDataRegFullFlag) ? (int)LPUART_ReadByte(LPUART4) : -1;
}

static void help(void)
{
    PRINTF("\r\nkeys: t test packets | f flood | c camera | x idle | i show last frame | r reset counters | h help\r\n");
    pump();
}

/* The last good frame as ASCII: 4x6-pixel blocks (a terminal cell is about twice as tall as
 * wide), mean brightness of each block mapped onto a 10-step ramp, dark to bright. */
static void show_frame(void)
{
    static const char ramp[] = " .:-=+*#%@";
    if (!g_frames_ok) {
        PRINTF("no camera frame yet: press c first\r\n");
        return;
    }
    PRINTF("\r\nlast good frame, %ux%u, mean brightness %u/255:\r\n", g_last_w, g_last_h, g_last_mean);
    for (uint32_t y = 0; y + 6 <= g_last_h; y += 6) {
        char line[96];
        uint32_t k = 0;
        for (uint32_t x = 0; x + 4 <= g_last_w && k < sizeof line - 3; x += 4) {
            uint32_t s = 0;
            for (uint32_t dy = 0; dy < 6; dy++)
                for (uint32_t dx = 0; dx < 4; dx++) s += g_last[(y + dy) * g_last_w + x + dx];
            line[k++] = ramp[(s / 24U) * 10U / 256U];
        }
        line[k++] = '\r';
        line[k++] = '\n';
        line[k] = 0;
        PRINTF("%s", line);
        pump();
    }
}

static void reset_counters(void)
{
    memset(&g_rx, 0, sizeof g_rx);
    g_overrun = g_framing = g_noise = g_ring_drop = 0;
    g_test_ok = g_pattern_err = 0;
    g_frames_ok = g_frames_crc = g_frames_incomplete = 0;
    g_fr.active = 0;
    PRINTF("counters reset\r\n");
}

/* One line per second. Rates are over the last second; counts are since boot (or since r). */
static void report(uint32_t dt_ms)
{
    static uint32_t last_bytes, last_ok, last_frames, last_errs;
    uint32_t bytes = g_rx.bytes - last_bytes, pkts = g_rx.ok - last_ok, frames = g_frames_ok - last_frames;
    last_bytes = g_rx.bytes;
    last_ok = g_rx.ok;
    last_frames = g_frames_ok;
    uint32_t bps = bytes * 1000U / dt_ms;
    uint32_t errs = link_rx_bad(&g_rx) + g_rx.lost + g_pattern_err + g_overrun + g_ring_drop;

    if (!g_rx.bytes) {
        PRINTF("[%4u s] nothing received yet. Check: ESP32 IO14 -> J8 pin 9, GND to GND, ESP32 running\r\n",
               g_ms / 1000U);
        return;
    }
    if (!g_rx.ok) {
        PRINTF("[%4u s] %u bytes arrived but no good packet: framing errors %u. Baud mismatch? "
               "Both sides must use %u\r\n", g_ms / 1000U, g_rx.bytes, g_framing, LINK_BAUD);
        return;
    }
    PRINTF("[%4u s] ESP32 %s mode %s | %3u pkt/s %5u B/s (%2u%% of link) | ok %u lost %u crc %u cobs %u long %u reboots %u",
           g_ms / 1000U, g_peer_alive ? "alive" : "GONE ", mode_name(g_peer_mode), pkts, bps,
           bps * 1000U / LINK_BAUD, g_rx.ok, g_rx.lost, g_rx.crc_err, g_rx.cobs_err, g_rx.too_long, g_rx.restarts);
    PRINTF(" | test ok %u pattern %u", g_test_ok, g_pattern_err);
    if (g_frames_ok || g_frames_crc || g_frames_incomplete)
        PRINTF(" | frames %u/s ok %u crc %u cut %u mean %u", frames, g_frames_ok, g_frames_crc,
               g_frames_incomplete, g_last_mean);
    PRINTF(" | uart ovr %u frm %u noise %u drop %u", g_overrun, g_framing, g_noise, g_ring_drop);
    PRINTF(" | ESP32 got ok %u bad %u lost %u\r\n", g_peer_ok, g_peer_bad, g_peer_lost);
    pump();
    leds(errs != last_errs, g_peer_alive, 0); /* red: something went wrong in the last second */
    last_errs = errs;
}

/* --------------------------------------------------------------------------------- main */
int main(void)
{
    BOARD_InitHardware();
    SysTick_Config(SystemCoreClock / 1000U);

    gpio_pin_config_t out = {kGPIO_DigitalOutput, 1};
    GPIO_PinInit(GPIO0, LED_R_PIN, &out);
    GPIO_PinInit(GPIO0, LED_G_PIN, &out);
    GPIO_PinInit(GPIO1, LED_B_PIN, &out);
    leds(0, 0, 0);

    lpuart_config_t cfg;
    LPUART_GetDefaultConfig(&cfg);
    cfg.baudRate_Bps = LINK_BAUD;
    cfg.enableTx = true;
    cfg.enableRx = true;
    status_t st = LPUART_Init(LINK_UART, &cfg, LINK_UART_CLK_FREQ);

    PRINTF("\r\n==== ESP32-CAM -> MCX N947 link test ====\r\n");
    PRINTF("link: LPUART0, RX = P0_12 (J8 pin 9) <- ESP32 IO14, TX = P0_13 (J8 pin 12) -> ESP32 IO13\r\n");
    PRINTF("baud %u, clock %u Hz, init %s\r\n", LINK_BAUD, LINK_UART_CLK_FREQ,
           st == kStatus_Success ? "OK" : "FAILED (baud not reachable)");
    help();

    LPUART_EnableInterrupts(LINK_UART, kLPUART_RxDataRegFullInterruptEnable | kLPUART_RxOverrunInterruptEnable |
                                           kLPUART_FramingErrorInterruptEnable | kLPUART_NoiseErrorInterruptEnable);
    EnableIRQ(LINK_IRQ);

    uint32_t t_hb = 0, t_report = g_ms, last_frames_ok = 0;
    for (;;) {
        pump();
        uint32_t now = g_ms;

        switch (console_getc()) {
            case 't': ask_mode(MODE_TEST); break;
            case 'f': ask_mode(MODE_FLOOD); break;
            case 'c': ask_mode(MODE_CAMERA); break;
            case 'x': ask_mode(MODE_IDLE); break;
            case 'i': show_frame(); break;
            case 'r': reset_counters(); break;
            case 'h': case '?': help(); break;
            default: break;
        }

        if (now - t_hb >= HEARTBEAT_MS) {
            t_hb = now;
            send_heartbeat();
        }

        if (g_peer_alive && now - g_last_hb > PEER_LOST_MS) {
            g_peer_alive = 0;
            PRINTF("ESP32 heartbeat: LOST (none for 1 s)\r\n");
        }

        if (g_frames_ok != last_frames_ok) { /* blue blink on every good frame */
            last_frames_ok = g_frames_ok;
            GPIO_PinWrite(GPIO1, LED_B_PIN, (g_frames_ok & 1U));
        }

        if (now - t_report >= 1000U) {
            report(now - t_report);
            t_report = now;
        }
    }
}
