/* LinkTest.ino: the ESP32-CAM side of the rover link test.
 *
 * It answers one question: can the ESP32-CAM send data to the MCX N947 reliably, and how fast?
 * It speaks the real rover protocol (rover_link.h: COBS packets, CRC-16, heartbeats), so what
 * passes here is the link the rover will use.
 *
 * Board: AI-Thinker ESP32-CAM. Arduino board "AI Thinker ESP32-CAM" (esp32:esp32:esp32cam).
 * Needs only the "esp32 by Espressif Systems" board package. The camera driver ships inside it,
 * so there is no extra library to install.
 *
 * Wiring (the full table and the reasons are in the README):
 *
 *   ESP32-CAM          MCX N947 (FRDM board)
 *   IO14  (TX)   --->  P0_12, header J8 pin 9    the MCU's UART receive
 *   IO13  (RX)   <---  P0_13, header J8 pin 12   the MCU's UART transmit
 *   GND          ----  GND                       ALWAYS. No common ground, no link.
 *
 * Never use IO12 for the link. It is a boot strapping pin: if the MCU's idle-high TX line holds
 * it high while the ESP32 powers up, the ESP32 picks the wrong flash voltage and will not boot.
 *
 * What it sends depends on the mode. The MCU picks the mode by sending a LINK_MODE packet
 * (keys t / f / c / x on the MCU console). The same keys typed into THIS board's Serial Monitor
 * work too, which helps when the MCU -> ESP32 wire is the broken one.
 *   x  idle    heartbeats only
 *   t  test    LINK_TEST packets, 50 per second, every length from 4 to 240 bytes   (default)
 *   f  flood   LINK_TEST packets back to back: the link's real throughput
 *   c  camera  160x120 grayscale frames, cut into chunks
 *
 * The USB Serial Monitor (115200) prints one status line per second, including what the MCU
 * says it received. Seeing those MCU numbers here proves both directions work.
 */
#include <stdarg.h>
#include "esp_camera.h"
#include "rover_link.h"

/* ---- link UART. LINK_BAUD must match LINK_BAUD in the MCU firmware (nxp/board/cm33_core0/app.h).
 * 921600 is the build plan's rate. If the MCU reports framing errors, drop BOTH sides to 115200. */
#define LINK_BAUD    921600
#define LINK_TX_PIN  14
#define LINK_RX_PIN  13
#define LED_PIN      33        /* small red LED on the back of the board, active low */

#define HEARTBEAT_MS 200       /* 5 Hz, as in the build plan */
#define PEER_LOST_MS 1000      /* no heartbeat for 1 s: the MCU is gone */
#define TEST_MS      20        /* 50 test packets per second in MODE_TEST */

/* ---- AI-Thinker ESP32-CAM camera pins (the same table as the CameraWebServer example) */
#define CAM_PWDN  32
#define CAM_XCLK  0
#define CAM_SIOD  26
#define CAM_SIOC  27
#define CAM_Y9    35
#define CAM_Y8    34
#define CAM_Y7    39
#define CAM_Y6    36
#define CAM_Y5    21
#define CAM_Y4    19
#define CAM_Y3    18
#define CAM_Y2    5
#define CAM_VSYNC 25
#define CAM_HREF  23
#define CAM_PCLK  22

HardwareSerial Link(1);

static link_rx_t g_rx;              /* everything received from the MCU */
static uint16_t g_seq;              /* seq of the next packet we send */
static uint8_t g_mode = MODE_TEST;
static uint32_t g_test_counter;     /* LINK_TEST packets sent */
static uint32_t g_tx_packets, g_tx_bytes;

/* what the MCU last told us in its heartbeat: how OUR packets are arriving there */
static uint32_t g_peer_ok, g_peer_bad, g_peer_lost;
static uint32_t g_last_hb_rx;       /* millis() of the last heartbeat from the MCU, 0 = never */
static bool g_peer_alive;

/* camera */
static bool g_cam_ok;
static camera_fb_t *g_fb;           /* frame being sent, NULL when none */
static uint32_t g_fb_off;
static uint16_t g_frame_id;
static uint32_t g_frames_sent;

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

/* ------------------------------------------------------------------------------ sending */
/* Free space in the UART transmit buffer. A packet is only written when it fits whole, so
 * loop() never blocks behind a slow UART and heartbeats keep flowing in flood and camera mode. */
static bool room_for_packet()
{
    return Link.availableForWrite() >= (int)LINK_WIRE_MAX;
}

static void send(uint8_t type, const uint8_t *payload, size_t n)
{
    uint8_t wire[LINK_WIRE_MAX];
    size_t m = link_pack(type, g_seq++, payload, n, wire);
    Link.write(wire, m);
    g_tx_packets++;
    g_tx_bytes += m;
}

/* One line of text for the MCU console. Mirrored to our own Serial Monitor. */
static void log_to_mcu(const char *fmt, ...)
{
    char s[LINK_MAX_PAYLOAD + 1];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s, sizeof s, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)LINK_MAX_PAYLOAD) n = LINK_MAX_PAYLOAD;
    Serial.printf("[to MCU] %s\n", s);
    send(LINK_LOG, (const uint8_t *)s, (size_t)n);
}

static void send_heartbeat()
{
    uint8_t p[LINK_HB_LEN];
    link_put_u32(p + 0, millis());
    link_put_u32(p + 4, g_rx.ok);
    link_put_u32(p + 8, link_rx_bad(&g_rx));
    link_put_u32(p + 12, g_rx.lost);
    p[16] = g_mode;
    send(LINK_HEARTBEAT, p, sizeof p);
}

static void send_test_packet()
{
    uint8_t p[LINK_MAX_PAYLOAD];
    size_t n = link_test_len(g_test_counter);
    link_put_u32(p, g_test_counter);
    for (size_t i = 4; i < n; i++)
        p[i] = link_test_byte(g_test_counter, (uint32_t)i);
    send(LINK_TEST, p, n);
    g_test_counter++;
}

/* ------------------------------------------------------------------------------- camera */
/* Safe to call again after a failure: every press of c retries. */
static bool camera_start()
{
    if (g_cam_ok) return true;

    camera_config_t c = {};
    c.pin_pwdn = CAM_PWDN;
    c.pin_reset = -1;
    c.pin_xclk = CAM_XCLK;
    c.pin_sccb_sda = CAM_SIOD;
    c.pin_sccb_scl = CAM_SIOC;
    c.pin_d7 = CAM_Y9;
    c.pin_d6 = CAM_Y8;
    c.pin_d5 = CAM_Y7;
    c.pin_d4 = CAM_Y6;
    c.pin_d3 = CAM_Y5;
    c.pin_d2 = CAM_Y4;
    c.pin_d1 = CAM_Y3;
    c.pin_d0 = CAM_Y2;
    c.pin_vsync = CAM_VSYNC;
    c.pin_href = CAM_HREF;
    c.pin_pclk = CAM_PCLK;
    c.xclk_freq_hz = 20000000;
    c.ledc_timer = LEDC_TIMER_0;
    c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = PIXFORMAT_GRAYSCALE; /* 1 byte per pixel: the MCU can check and use it raw */
    c.frame_size = FRAMESIZE_QQVGA;       /* 160 x 120 = 19200 bytes, ~0.21 s at 921600 baud */
    c.jpeg_quality = 12;
    c.fb_count = psramFound() ? 2 : 1;
    c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;     /* always send the newest frame, never a stale one */

    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) {
        log_to_mcu("camera init FAILED 0x%x (%s): reseat the ribbon cable, power the ESP32-CAM from 5 V",
                   (unsigned)err, esp_err_to_name(err));
        return g_cam_ok = false;
    }
    log_to_mcu("camera up: 160x120 grayscale, psram %s", psramFound() ? "yes" : "no");
    return g_cam_ok = true;
}

static void camera_abort()
{
    if (g_fb) {
        esp_camera_fb_return(g_fb);
        g_fb = NULL;
    }
}

/* Called every loop() in camera mode. Sends whatever fits in the UART buffer right now:
 * a FRAME_START when a new frame begins, then one FRAME_CHUNK at a time. */
static void camera_pump()
{
    if (!g_cam_ok) return;
    if (!g_fb) {
        if (!room_for_packet()) return;
        g_fb = esp_camera_fb_get();
        if (!g_fb) return;
        g_fb_off = 0;
        g_frame_id++;
        uint8_t p[LINK_FS_LEN];
        link_put_u16(p + 0, g_frame_id);
        link_put_u16(p + 2, (uint16_t)g_fb->width);
        link_put_u16(p + 4, (uint16_t)g_fb->height);
        p[6] = LINK_FMT_GRAY8;
        link_put_u32(p + 7, (uint32_t)g_fb->len);
        link_put_u32(p + 11, link_crc32(0, g_fb->buf, g_fb->len));
        send(LINK_FRAME_START, p, sizeof p);
    }
    while (g_fb && room_for_packet()) {
        uint8_t p[LINK_MAX_PAYLOAD];
        uint32_t n = g_fb->len - g_fb_off;
        if (n > LINK_CHUNK_DATA) n = LINK_CHUNK_DATA;
        link_put_u16(p + 0, g_frame_id);
        link_put_u32(p + 2, g_fb_off);
        memcpy(p + LINK_CHUNK_HDR, g_fb->buf + g_fb_off, n);
        send(LINK_FRAME_CHUNK, p, LINK_CHUNK_HDR + n);
        g_fb_off += n;
        if (g_fb_off >= g_fb->len) {
            camera_abort(); /* frame fully sent: hand the buffer back */
            g_frames_sent++;
        }
    }
}

/* ---------------------------------------------------------------------------- receiving */
static void set_mode(uint8_t m, const char *who)
{
    if (m > MODE_CAMERA) return;
    if (m != MODE_CAMERA) camera_abort();
    g_mode = m;
    Serial.printf("mode -> %s (asked by %s)\n", mode_name(m), who);
    if (m == MODE_CAMERA) camera_start();
}

static void on_packet()
{
    switch (g_rx.type) {
        case LINK_HEARTBEAT:
            if (g_rx.len >= LINK_HB_LEN) {
                g_peer_ok = link_get_u32(g_rx.payload + 4);
                g_peer_bad = link_get_u32(g_rx.payload + 8);
                g_peer_lost = link_get_u32(g_rx.payload + 12);
            }
            g_last_hb_rx = millis();
            if (!g_peer_alive) Serial.println("MCU heartbeat: FOUND");
            g_peer_alive = true;
            {
                static bool led;
                led = !led;
                digitalWrite(LED_PIN, led ? LOW : HIGH); /* blinks at 2.5 Hz while the MCU talks */
            }
            break;
        case LINK_MODE:
            if (g_rx.len >= 1) set_mode(g_rx.payload[0], "MCU");
            break;
        default:
            break; /* the rest of the rover protocol is not used by this test */
    }
}

/* ------------------------------------------------------------------------------- arduino */
void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); /* off */

    Link.setRxBufferSize(2048);
    Link.setTxBufferSize(4096);  /* lets loop() queue several packets and get on with its work */
    Link.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
    Link.write((uint8_t)0);      /* ends any junk the MCU caught while we booted (IO14 toggles at reset) */

    delay(100);
    Serial.printf("\nESP32-CAM link test. Link UART: TX IO%d, RX IO%d, %d baud.\n", LINK_TX_PIN,
                  LINK_RX_PIN, LINK_BAUD);
    Serial.println("Keys here (then Enter): t test, f flood, c camera, x idle.");
    log_to_mcu("ESP32-CAM LinkTest up, %d baud, psram %s, mode %s", LINK_BAUD,
               psramFound() ? "yes" : "no", mode_name(g_mode));
}

void loop()
{
    static uint32_t t_hb, t_test, t_print, last_pkts, last_bytes;
    uint32_t now = millis();

    while (Link.available() > 0)
        if (link_rx_byte(&g_rx, (uint8_t)Link.read())) on_packet();

    while (Serial.available() > 0) {
        switch (Serial.read()) {
            case 't': set_mode(MODE_TEST, "keyboard"); break;
            case 'f': set_mode(MODE_FLOOD, "keyboard"); break;
            case 'c': set_mode(MODE_CAMERA, "keyboard"); break;
            case 'x': set_mode(MODE_IDLE, "keyboard"); break;
        }
    }

    if (now - t_hb >= HEARTBEAT_MS && room_for_packet()) {
        t_hb = now;
        send_heartbeat();
    }

    if (g_mode == MODE_TEST && now - t_test >= TEST_MS && room_for_packet()) {
        t_test = now;
        send_test_packet();
    } else if (g_mode == MODE_FLOOD) {
        while (room_for_packet()) send_test_packet();
    } else if (g_mode == MODE_CAMERA) {
        camera_pump();
    }

    if (g_peer_alive && now - g_last_hb_rx > PEER_LOST_MS) {
        g_peer_alive = false;
        Serial.println("MCU heartbeat: LOST (none for 1 s). Check IO13 <- J8 pin 12 and GND.");
    }

    if (now - t_print >= 1000) {
        uint32_t dt = now - t_print;
        t_print = now;
        uint32_t pps = (g_tx_packets - last_pkts) * 1000u / dt;
        uint32_t bps = (g_tx_bytes - last_bytes) * 1000u / dt;
        last_pkts = g_tx_packets;
        last_bytes = g_tx_bytes;
        Serial.printf("[%5lus] %-6s sent %4lu pkt/s %6lu B/s (%lu%% of link)", (unsigned long)(now / 1000),
                      mode_name(g_mode), (unsigned long)pps, (unsigned long)bps,
                      (unsigned long)(bps * 10u * 100u / LINK_BAUD));
        if (g_mode == MODE_CAMERA) Serial.printf(" frames %lu", (unsigned long)g_frames_sent);
        if (g_peer_alive)
            Serial.printf(" | MCU got: ok %lu bad %lu lost %lu\n", (unsigned long)g_peer_ok,
                          (unsigned long)g_peer_bad, (unsigned long)g_peer_lost);
        else
            Serial.printf(" | no heartbeat from the MCU (rx bytes %lu, bad %lu)\n",
                          (unsigned long)g_rx.bytes, (unsigned long)link_rx_bad(&g_rx));
    }
}
