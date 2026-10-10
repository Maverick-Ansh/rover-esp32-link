#!/usr/bin/env python3
"""link_tool.py: the PC side of the ESP32-CAM <-> MCX N947 link test.  Needs: pip install pyserial

    python tools/link_tool.py ports                   list serial ports, marks the MCU-Link VCOM
    python tools/link_tool.py monitor [--port COM8]   the MCX N947 console: shows its output, sends your keys
    python tools/link_tool.py fake-esp32 --port COM5  a USB-TTL adapter wired to J8 pretends to be the ESP32-CAM
    python tools/link_tool.py fake-mcu --port COM5    a USB-TTL adapter wired to IO13/IO14 pretends to be the MCU
    python tools/link_tool.py selftest [stream.bin]   checks this file against the C protocol

The two fake-* modes split a broken link in half. If the real pair does not talk:
  fake-esp32 + real MCX N947 works  -> the MCU side is fine, look at the ESP32 (sketch, IO14, power)
  fake-mcu   + real ESP32-CAM works -> the ESP32 side is fine, look at the MCU (firmware, J8 pins)

This file is a line-by-line twin of esp32cam/LinkTest/rover_link.h. `selftest stream.bin` proves
the twin: it re-encodes the packets the C code wrote and must get the same bytes back.
"""
import argparse, binascii, os, struct, sys, threading, time

# ------------------------------------------------------------------ protocol (rover_link.h)
MAX_PAYLOAD = 240
RAW_MAX = 3 + MAX_PAYLOAD + 2
WIRE_MAX = RAW_MAX + RAW_MAX // 254 + 2
TELEMETRY, EVENT, SKILL_CMD, SKILL_STATUS, HEARTBEAT = 0x01, 0x02, 0x03, 0x04, 0x05
TEST, FRAME_START, FRAME_CHUNK, MODE, LOG = 0x10, 0x11, 0x12, 0x13, 0x14
MODE_IDLE, MODE_TEST, MODE_FLOOD, MODE_CAMERA = 0, 1, 2, 3
MODE_NAMES = {0: "IDLE", 1: "TEST", 2: "FLOOD", 3: "CAMERA"}
KEY_MODES = {"x": MODE_IDLE, "t": MODE_TEST, "f": MODE_FLOOD, "c": MODE_CAMERA}
CHUNK_HDR = 6
CHUNK_DATA = MAX_PAYLOAD - CHUNK_HDR


def crc16(data, c=0xFFFF):
    return binascii.crc_hqx(data, c)          # CRC-16/CCITT-FALSE when started at 0xFFFF


def crc32(data, c=0):
    return binascii.crc32(data, c) & 0xFFFFFFFF


def cobs_encode(data):
    out, code_at, code = bytearray([0]), 0, 1
    for b in data:
        if b == 0:
            out[code_at] = code
            code_at, code = len(out), 1
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_at] = code
                code_at, code = len(out), 1
                out.append(0)
    out[code_at] = code
    return bytes(out)


def cobs_decode(enc, limit=RAW_MAX):
    out, i, n = bytearray(), 0, len(enc)
    while i < n:
        code = enc[i]
        i += 1
        if code == 0 or i + code - 1 > n:
            return None
        out += enc[i:i + code - 1]
        i += code - 1
        if code != 0xFF and i < n:
            out.append(0)
        if len(out) > limit:
            return None
    return bytes(out)


def pack(ptype, seq, payload=b""):
    raw = bytes([ptype]) + struct.pack("<H", seq & 0xFFFF) + bytes(payload[:MAX_PAYLOAD])
    return cobs_encode(raw + struct.pack("<H", crc16(raw))) + b"\x00"


def test_byte(counter, i):
    return 0 if (counter + i) % 5 == 0 else (counter * 31 + i * 7) & 0xFF


def test_len(counter):
    return 4 + counter % (MAX_PAYLOAD - 4 + 1)


def test_payload(counter):
    return struct.pack("<I", counter) + bytes(test_byte(counter, i) for i in range(4, test_len(counter)))


class Rx:
    """link_rx_t: feed bytes, get back (type, seq, payload) for every good packet."""

    def __init__(self):
        self.enc, self.overflow = bytearray(), False
        self.bytes = self.ok = self.crc_err = self.cobs_err = self.too_long = self.lost = self.restarts = 0
        self.next_seq = None

    @property
    def bad(self):
        return self.crc_err + self.cobs_err + self.too_long

    def feed(self, data):
        for b in data:
            self.bytes += 1
            if b:
                if len(self.enc) < WIRE_MAX:
                    self.enc.append(b)
                else:
                    self.overflow = True
                continue
            if not self.enc and not self.overflow:
                continue
            enc, overflow = bytes(self.enc), self.overflow
            self.enc, self.overflow = bytearray(), False
            if overflow:
                self.too_long += 1
                continue
            raw = cobs_decode(enc)
            if raw is None or len(raw) < 5:
                self.cobs_err += 1
                continue
            if crc16(raw[:-2]) != struct.unpack("<H", raw[-2:])[0]:
                self.crc_err += 1
                continue
            seq = struct.unpack("<H", raw[1:3])[0]
            if self.next_seq is not None:
                gap = (seq - self.next_seq) & 0xFFFF
                if gap < 2000:
                    self.lost += gap
                else:
                    self.restarts += 1
            self.next_seq = (seq + 1) & 0xFFFF
            self.ok += 1
            yield raw[0], seq, raw[3:-2]


def heartbeat(rx, mode):
    return struct.pack("<IIIIB", int(time.monotonic() * 1000) & 0xFFFFFFFF, rx.ok, rx.bad, rx.lost, mode & 0xFF)


# ------------------------------------------------------------------------------ serial helpers
def need_serial():
    try:
        import serial  # noqa: F401
        from serial.tools import list_ports  # noqa: F401
    except ImportError:
        sys.exit("pyserial is missing: pip install pyserial")


def is_mcu_link(p):
    return (p.vid, p.pid) == (0x1FC9, 0x0143) or "MCU-LINK" in (p.description or "").upper()


def cmd_ports(_):
    need_serial()
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        print("no serial ports. Plug the board in (a data cable, not a charge-only one).")
    for p in ports:
        tag = "   <- MCX N947 console (MCU-Link)" if is_mcu_link(p) else ""
        print(f"{p.device:10s} {p.description}{tag}")


def find_mcu_link():
    from serial.tools import list_ports
    for p in list_ports.comports():
        if is_mcu_link(p):
            return p.device
    sys.exit("MCU-Link VCOM not found. Is the FRDM board plugged in by its MCU-Link USB port (J17)? "
             "Give --port by hand, see `ports`.")


class Keys:
    """Single key presses without Enter, on Windows and on Linux/macOS."""

    def __enter__(self):
        if os.name != "nt" and sys.stdin.isatty():
            import termios, tty
            self.old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *a):
        if os.name != "nt" and sys.stdin.isatty():
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old)

    def get(self):
        """A pressed key, or None. Never blocks."""
        if os.name == "nt":
            import msvcrt
            return msvcrt.getwch() if msvcrt.kbhit() else None
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


# ------------------------------------------------------------------------------------ monitor
def cmd_monitor(a):
    need_serial()
    import serial
    port = a.port or find_mcu_link()
    log = open(a.log, "a", encoding="utf-8") if a.log else None
    print(f"--- {port} @ 115200. Keys go to the board (t f c x i r h). Ctrl+C quits. ---")
    with serial.Serial(port, 115200, timeout=0.05) as s, Keys() as keys:
        stop = threading.Event()

        def reader():
            while not stop.is_set():
                data = s.read(512)
                if data:
                    text = data.decode("utf-8", "replace")
                    sys.stdout.write(text)
                    sys.stdout.flush()
                    if log:
                        log.write(text.replace("\r", ""))
                        log.flush()

        th = threading.Thread(target=reader, daemon=True)
        th.start()
        try:
            while True:
                k = keys.get()
                if k:
                    s.write(k.encode("ascii", "ignore"))
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        stop.set()
        th.join(1)
    print("\n--- end ---")


# --------------------------------------------------------------------------------- fake-esp32
def cmd_fake_esp32(a):
    """Plays the ESP32-CAM: heartbeats, test packets, flood, and a synthetic camera frame."""
    need_serial()
    import serial
    rx, seq, mode, counter, frame_id = Rx(), 0, MODE_TEST, 0, 0
    sent = sent_bytes = 0
    peer = None

    def send(s, ptype, payload=b""):
        nonlocal seq, sent, sent_bytes
        w = pack(ptype, seq, payload)
        seq += 1
        s.write(w)
        sent += 1
        sent_bytes += len(w)

    with serial.Serial(a.port, a.baud, timeout=0, write_timeout=2) as s, Keys() as keys:
        print(f"--- pretending to be the ESP32-CAM on {a.port} @ {a.baud}. "
              "Wire: adapter TX -> J8 pin 9, adapter RX <- J8 pin 12, GND. Keys: t f c x, Ctrl+C quits ---")
        s.write(b"\x00")
        send(s, LOG, b"fake ESP32 (link_tool.py) up")
        t_hb = t_test = t_print = time.monotonic()
        last = (0, 0)
        try:
            while True:
                now = time.monotonic()
                for ptype, _, p in rx.feed(s.read(4096)):
                    if ptype == HEARTBEAT and len(p) >= 17:
                        peer = struct.unpack("<IIIIB", p[:17])
                    elif ptype == MODE and p:
                        mode = p[0]
                        print(f"MCU asked for mode {MODE_NAMES.get(mode, mode)}")
                k = keys.get()
                if k in KEY_MODES:
                    mode = KEY_MODES[k]
                    print(f"mode -> {MODE_NAMES[mode]} (keyboard)")
                if now - t_hb >= 0.2:
                    t_hb = now
                    send(s, HEARTBEAT, heartbeat(rx, mode))
                if mode == MODE_TEST:  # 50 per second on average, even with a coarse OS sleep
                    if now - t_test > 0.1:
                        t_test = now - 0.02
                    while now - t_test >= 0.02:
                        t_test += 0.02
                        send(s, TEST, test_payload(counter))
                        counter += 1
                elif mode == MODE_FLOOD:
                    for _ in range(8):
                        send(s, TEST, test_payload(counter))
                        counter += 1
                elif mode == MODE_CAMERA:  # a diagonal gradient that slides: the MCU's ASCII view moves
                    frame_id = (frame_id + 1) & 0xFFFF
                    w, h = 160, 120
                    img = bytes(((x + y + frame_id * 8) * 2) & 0xFF for y in range(h) for x in range(w))
                    send(s, FRAME_START, struct.pack("<HHHBII", frame_id, w, h, 0, len(img), crc32(img)))
                    for off in range(0, len(img), CHUNK_DATA):
                        send(s, FRAME_CHUNK, struct.pack("<HI", frame_id, off) + img[off:off + CHUNK_DATA])
                if now - t_print >= 1:
                    dt, t_print = now - t_print, now
                    pps, bps = (sent - last[0]) / dt, (sent_bytes - last[1]) / dt
                    last = (sent, sent_bytes)
                    msg = (f"MCU got: ok {peer[1]} bad {peer[2]} lost {peer[3]}" if peer else
                           f"no heartbeat from the MCU yet (rx bytes {rx.bytes}, bad {rx.bad})")
                    print(f"{MODE_NAMES.get(mode, mode):6s} sent {pps:5.0f} pkt/s {bps:7.0f} B/s | {msg}")
                time.sleep(0.001)
        except KeyboardInterrupt:
            pass


# ----------------------------------------------------------------------------------- fake-mcu
def cmd_fake_mcu(a):
    """Plays the MCX N947: checks everything the ESP32 sends, heartbeats back, keys pick the mode."""
    need_serial()
    import serial
    rx, seq = Rx(), 0
    test_ok = pattern_err = frames_ok = frames_bad = 0
    fr = None
    last_img = None
    peer_mode = None

    with serial.Serial(a.port, a.baud, timeout=0) as s, Keys() as keys:
        print(f"--- pretending to be the MCX N947 on {a.port} @ {a.baud}. "
              "Wire: adapter RX <- ESP32 IO14, adapter TX -> ESP32 IO13, GND. "
              "Keys: t f c x, i = show frame, Ctrl+C quits ---")
        t_hb = t_print = time.monotonic()
        last = (0, 0)
        try:
            while True:
                now = time.monotonic()
                for ptype, _, p in rx.feed(s.read(8192)):
                    if ptype == HEARTBEAT and len(p) >= 17:
                        peer_mode = p[16]
                    elif ptype == TEST and len(p) >= 4:
                        c = struct.unpack("<I", p[:4])[0]
                        if p == test_payload(c):
                            test_ok += 1
                        else:
                            pattern_err += 1
                    elif ptype == LOG:
                        print("ESP32 says:", p.decode("utf-8", "replace"))
                    elif ptype == FRAME_START and len(p) >= 15:
                        if fr:
                            frames_bad += 1
                        fid, w, h, fmt, total, crc = struct.unpack("<HHHBII", p[:15])
                        fr = dict(id=fid, w=w, h=h, total=total, crc=crc, data=bytearray())
                    elif ptype == FRAME_CHUNK and fr and len(p) >= CHUNK_HDR:
                        fid, off = struct.unpack("<HI", p[:CHUNK_HDR])
                        if fid != fr["id"] or off != len(fr["data"]):
                            frames_bad += 1
                            fr = None
                            continue
                        fr["data"] += p[CHUNK_HDR:]
                        if len(fr["data"]) >= fr["total"]:
                            if crc32(bytes(fr["data"])) == fr["crc"]:
                                frames_ok += 1
                                last_img = (fr["w"], fr["h"], bytes(fr["data"]))
                            else:
                                frames_bad += 1
                            fr = None
                k = keys.get()
                if k in KEY_MODES:
                    s.write(pack(MODE, seq, bytes([KEY_MODES[k]])))
                    seq += 1
                    print(f"asked the ESP32 for mode {MODE_NAMES[KEY_MODES[k]]}")
                elif k == "i":
                    show_ascii(last_img)
                if now - t_hb >= 0.2:
                    t_hb = now
                    s.write(pack(HEARTBEAT, seq, heartbeat(rx, 0xFF)))
                    seq += 1
                if now - t_print >= 1:
                    dt, t_print = now - t_print, now
                    pps, bps = (rx.ok - last[0]) / dt, (rx.bytes - last[1]) / dt
                    last = (rx.ok, rx.bytes)
                    print(f"ESP32 mode {MODE_NAMES.get(peer_mode, '?'):6s} | {pps:5.0f} pkt/s {bps:7.0f} B/s"
                          f" ({100 * bps * 10 / a.baud:3.0f}% of link) | ok {rx.ok} lost {rx.lost}"
                          f" crc {rx.crc_err} cobs {rx.cobs_err} long {rx.too_long} | test ok {test_ok}"
                          f" pattern {pattern_err} | frames ok {frames_ok} bad {frames_bad}")
                time.sleep(0.001)
        except KeyboardInterrupt:
            pass


def show_ascii(img):
    if not img:
        print("no frame yet: press c first")
        return
    w, h, data = img
    ramp = " .:-=+*#%@"
    print(f"last frame {w}x{h}, mean {sum(data) // len(data)}")
    for y in range(0, h - 5, 6):
        print("".join(ramp[sum(data[(y + dy) * w + x + dx] for dy in range(6) for dx in range(4)) // 24 * 10 // 256]
                      for x in range(0, w - 3, 4)))


# ----------------------------------------------------------------------------------- selftest
def cmd_selftest(a):
    fails = 0

    def check(ok, what):
        nonlocal fails
        if not ok:
            fails += 1
            print("FAIL:", what)

    check(crc16(b"123456789") == 0x29B1, "crc16 check value")
    check(crc32(b"123456789") == 0xCBF43926, "crc32 check value")
    for n in range(0, 700):
        for data in (os.urandom(n), bytes(1 + (i % 255) for i in range(n)), bytes(n)):
            check(cobs_decode(cobs_encode(data), limit=10**6) == data, f"cobs round trip n={n}")
    if a.stream:  # written by: test_link --dump stream.bin
        blob = open(a.stream, "rb").read()
        mine = b"".join(pack(TEST, k, test_payload(k)) for k in range(600))
        mine += pack(LOG, 600, b"hello from C") + pack(HEARTBEAT, 601, struct.pack("<IIIIB", 1, 2, 3, 4, MODE_CAMERA))
        check(blob == mine, f"Python encodes differently from C ({len(mine)} vs {len(blob)} bytes)")
        rx = Rx()
        got = list(rx.feed(blob))
        check(len(got) == 602 and rx.bad == 0 and rx.lost == 0, f"decoded {len(got)} packets, bad {rx.bad}")
        check(all(p == test_payload(k) for k, (_, _, p) in enumerate(got[:600])), "test payloads differ")
        print(f"C stream: {len(blob)} bytes, {len(got)} packets, byte-identical to Python: {blob == mine}")
    print("selftest:", "FAILED" if fails else "all passed")
    sys.exit(1 if fails else 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports").set_defaults(fn=cmd_ports)
    m = sub.add_parser("monitor")
    m.add_argument("--port")
    m.add_argument("--log", help="also append everything to this file")
    m.set_defaults(fn=cmd_monitor)
    for name, fn in (("fake-esp32", cmd_fake_esp32), ("fake-mcu", cmd_fake_mcu)):
        f = sub.add_parser(name)
        f.add_argument("--port", required=True, help="the USB-TTL adapter's port")
        f.add_argument("--baud", type=int, default=921600)
        f.set_defaults(fn=fn)
    t = sub.add_parser("selftest")
    t.add_argument("stream", nargs="?")
    t.set_defaults(fn=cmd_selftest)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
