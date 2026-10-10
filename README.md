# rover-esp32-link

Bring-up test for the rover's **ESP32-CAM → NXP MCX N947** link. It answers one question:
can the ESP32-CAM send data to the MCX N947 reliably, and how fast?

The two boards talk over a UART at **921600 baud** with the protocol the rover will actually use
(build plan, Level 4: COBS-framed packets, CRC-16, sequence numbers, 5 Hz heartbeats both ways).
The test sends four kinds of traffic and checks every byte:

| key | what the ESP32 sends | what it proves |
|---|---|---|
| `t` | test packets, 50 per second, every length from 4 to 240 bytes | packets arrive intact, in order |
| `f` | test packets back to back | the real throughput of the link |
| `c` | live camera frames, 160×120 grayscale, checked with CRC-32 | real camera data gets across whole |
| `x` | heartbeats only | the idle link stays up |

The MCX N947 also sends heartbeats back, carrying what it received, so **both directions** are
tested at the same time.

```
esp32cam/LinkTest/LinkTest.ino    the ESP32-CAM sketch (Arduino)
esp32cam/LinkTest/rover_link.h    THE protocol, shared by both boards (the only copy to edit)
nxp/prebuilt/esp_link_cm33_core0.elf   ready-to-flash MCX N947 firmware (no SDK needed)
nxp/app/, nxp/board/              its source (MCUXpresso SDK example), nxp/install.py
tools/link_tool.py                PC helper: console, and fake-ESP32 / fake-MCU for fault finding
test/test_link.c                  host test of the protocol (no hardware)
```

---

## 1. What you need

**Hardware**

- FRDM-MCXN947 board + a USB-C **data** cable (a charge-only cable shows no device at all)
- ESP32-CAM (AI-Thinker) with its OV2640 camera, **no microSD card inserted**
  (the SD slot shares IO13 / IO14 with the link)
- One way to program the ESP32-CAM:
  - the **ESP32-CAM-MB** USB board it usually ships on, or
  - a **USB-TTL adapter** (CH340, CP2102 or FTDI) with a 5 V pin
- 3 jumper wires for the link (5 if the ESP32-CAM is powered from the FRDM board)

**Software** (all free; Windows commands shown, Linux/macOS notes at the end)

| what | why | install |
|---|---|---|
| Python 3.10+ | flashing the NXP board, the PC tool | python.org, tick **"Add python.exe to PATH"** |
| pyOCD + pyserial | flash the MCX N947 through its on-board MCU-Link | `pip install pyocd pyserial` |
| MCX N947 device pack | tells pyOCD how to program this chip | `pyocd pack update` then `pyocd pack install mcxn947vdf` |
| Arduino IDE 2.x | build and upload the ESP32-CAM sketch | arduino.cc/en/software |
| **esp32** board package by Espressif, 3.x | ESP32 compiler + the camera driver | see below |
| CH340 driver | only if the ESP32-CAM-MB shows no COM port | Windows Update normally installs it |

**There are no Arduino libraries to install.** The camera driver (`esp_camera.h`) is part of the
esp32 board package. The sketch needs nothing else.

Board package, once:

1. Arduino IDE → **File → Preferences → Additional boards manager URLs**, add
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. **Tools → Board → Boards Manager**, search `esp32`, install **esp32 by Espressif Systems**
   (tested with 3.3.12). It is a large download (~1 GB), give it time.

The MCU-Link on the FRDM board needs no driver on Windows 10/11. Check it with:

```powershell
pyocd list
```

Expected: one probe, `MCU-LINK FRDM-MCXN947`. It may print the target as `mcxn947vdft` with a red
cross; that is a pyOCD quirk, ignore it (see the trap in section 3).

---

## 2. Get the code

```powershell
git clone https://github.com/Maverick-Ansh/rover-esp32-link
cd rover-esp32-link
```

(or **Code → Download ZIP** on GitHub and unzip it). Every command below runs from this folder.

---

## 3. Flash the MCX N947 (2 minutes, no SDK needed)

Plug the FRDM board in by its **MCU-Link** USB-C port (J17, the one labelled MCU-Link).

```powershell
$env:PYTHONIOENCODING = "utf-8"          # cmd.exe: set PYTHONIOENCODING=utf-8
pyocd flash -t mcxn947vdf nxp/prebuilt/esp_link_cm33_core0.elf
pyocd reset -t mcxn947vdf
```

Two traps, both real:

- **Always pass `-t mcxn947vdf`** (no trailing T). `pyocd list` reports `mcxn947vdft`, which pyOCD
  itself then rejects with "Target type mcxn947vdft not recognized".
- **Set `PYTHONIOENCODING=utf-8` first** on Windows, or pyOCD crashes with `UnicodeEncodeError`.

Now open the board's console:

```powershell
python tools/link_tool.py monitor --log run1.txt
```

It finds the MCU-Link COM port by itself, shows the board's output, sends your key presses to the
board, and saves everything to `run1.txt`. (Any serial terminal works too: the MCU-Link COM port at
115200. In the Arduino Serial Monitor, type the key and press Enter.) You should see:

```
==== ESP32-CAM -> MCX N947 link test ====
link: LPUART0, RX = P0_12 (J8 pin 9) <- ESP32 IO14, TX = P0_13 (J8 pin 12) -> ESP32 IO13
baud 921600, clock 12000000 Hz, init OK
keys: t test packets | f flood | c camera | x idle | i show last frame | r reset counters | h help
[   1 s] nothing received yet. Check: ESP32 IO14 -> J8 pin 9, GND to GND, ESP32 running
```

That last line repeats every second until the ESP32 is wired. That is expected.

---

## 4. Flash the ESP32-CAM

1. Open `esp32cam/LinkTest/LinkTest.ino` in the Arduino IDE. Keep the folder name `LinkTest` and
   keep `rover_link.h` next to it (the IDE shows it as a second tab).
2. **Tools → Board → esp32 → AI Thinker ESP32-CAM**. **Tools → Port** → the ESP32's COM port
   (CH340 / USB-SERIAL). Leave every other Tools option at its default.
3. Upload.
   - **ESP32-CAM-MB:** just press Upload. If it says `Failed to connect to ESP32`, hold the **IO0**
     button on the MB board, tap **RST**, release IO0, upload again.
   - **USB-TTL adapter:** adapter 5V → 5V, GND → GND, adapter TX → U0R, adapter RX → U0T. Put a
     jumper from **IO0 to GND**, press RST, upload, **remove the jumper**, press RST again. (With
     IO0 still grounded it boots back into the loader and looks dead.)
4. **Tools → Serial Monitor**, 115200 baud. Expected every second:

```
ESP32-CAM link test. Link UART: TX IO14, RX IO13, 921600 baud.
[    3s] TEST   sent   55 pkt/s   6900 B/s (7% of link) | no heartbeat from the MCU (rx bytes 0, bad 0)
```

It is already sending; nobody is listening yet. That is expected.

Command line instead of the IDE (optional): `arduino-cli compile --fqbn esp32:esp32:esp32cam esp32cam/LinkTest`
then `arduino-cli upload -p COM5 --fqbn esp32:esp32:esp32cam esp32cam/LinkTest`.

---

## 5. Wire the two boards

**Unplug both boards while wiring.** Both run 3.3 V logic, so the wires go straight across:

| ESP32-CAM pin | → | FRDM-MCXN947 | what it carries |
|---|---|---|---|
| **IO14** | → | **J8 pin 9** (P0_12) | ESP32 transmits, MCU receives |
| **IO13** | ← | **J8 pin 12** (P0_13) | MCU transmits, ESP32 receives |
| **GND** | — | **GND** (any GND pin) | the shared reference. **Without it nothing works** |

Finding the pins:

- **J8** on the FRDM board is the 2×14-pin header meant for the parallel LCD. Pin 1 is marked on
  the board. Pins are numbered across in pairs (1-2, 3-4, …), so odd pins run along one row and even
  pins along the other: **pin 9 is the 5th pin of the odd row, pin 12 is the 6th pin of the even
  row**, both counted from the pin-1 end. If in doubt, the J8 table is in the *FRDM-MCXN947 Board
  User Manual* (nxp.com, the board's Documentation tab).
- On the ESP32-CAM the pins are printed next to the header: IO14 and IO13 are on the same side as
  5V, GND, IO12, IO15, IO2, IO4.

Rules that will bite otherwise:

- **Never use IO12** for the link. It is a boot strapping pin; held high at power-up (which the
  MCU's idle UART line does) it makes the ESP32 choose the wrong flash voltage and it will not boot.
- **Cross the wires**: one side's TX goes to the other side's RX. TX to TX does nothing.
- **Keep the wires short** (under ~20 cm) at 921600 baud.

Powering the ESP32-CAM while it is wired:

- **USB-TTL adapter:** keep it connected (5V, GND, U0R, U0T, IO0 jumper **removed**). It powers the
  ESP32 and keeps its Serial Monitor working. Add the 3 link wires above. This is the best setup.
- **ESP32-CAM-MB:** the MB board covers the IO13/IO14 pins, so after uploading take the ESP32-CAM
  off the MB and power it from the FRDM board instead: ESP32 **5V → the FRDM's 5V pin**, ESP32
  **GND → GND**. You lose the ESP32's own Serial Monitor, but nothing important: the ESP32 forwards
  its messages to the MCU console (`ESP32 says: ...`).
- If the ESP32 keeps rebooting (the `reboots` counter climbs, or `ESP32 says: ESP32-CAM LinkTest up`
  keeps reappearing), especially when the camera starts, its 5 V supply is too weak. Give it its own
  5 V / 2 A supply, **grounds still tied together**.

---

## 6. Run the test

Plug in the FRDM board, start `python tools/link_tool.py monitor --log run1.txt`, then power the
ESP32-CAM.

**Step 1: test packets (the ESP32 starts in this mode by itself).** Within a second or two:

```
ESP32 heartbeat: FOUND
[  12 s] ESP32 alive mode TEST | 55 pkt/s 6912 B/s ( 7% of link) | ok 612 lost 0 crc 0 cobs 1 long 0 reboots 0 | test ok 551 pattern 0 | uart ovr 0 frm 0 noise 0 drop 0 | ESP32 got ok 58 bad 0 lost 0
```

55 packets per second = 50 test packets + 5 heartbeats. One or two `cobs`/`crc`/`frm` errors **right
when the ESP32 powers up are normal**: its IO14 pin wiggles during boot and the MCU sees that junk.
They must not keep climbing. The ESP32's red LED (on the back) blinks: it is receiving the MCU's
heartbeats. Its Serial Monitor (if connected) shows `MCU heartbeat: FOUND` and
`MCU got: ok … bad 0 lost 0`.

**Step 2: throughput.** Press `f`. The ESP32 floods. Expect roughly **85-95% of link**, about
80-90 KB/s. Leave it for **5 minutes**. `lost`, `crc`, `pattern`, `ovr`, `drop` must all stay at 0
(or stay at the boot value).

**Step 3: camera.** Press `c`. Expect `ESP32 says: camera up: 160x120 grayscale, psram yes`, then
a `frames` section: about **4 frames per second** at 921600 baud, `ok` climbing, `crc 0`, `cut 0`.
Press `i`: the last frame prints as ASCII art. Point the camera at something bright, press `i`;
cover the lens, press `i` again. The picture and the `mean` brightness change. That is real camera
data crossing the link byte for byte (each frame carries a CRC-32 the MCU recomputes).

**Step 4: idle.** Press `x`. Traffic drops to about 5 packets/s (heartbeats only); the link stays
`alive`.

**Step 5: recovery.** Press `t`. Pull the IO14 wire out for 3 seconds and put it back. The console
says `ESP32 heartbeat: LOST (none for 1 s)`, then `FOUND`; `lost` jumps by the packets that went
missing and then stops growing. Press the ESP32's RST button: `reboots` goes up by one and the link
comes back by itself.

**It PASSES when**, after Step 2's 5 minutes and Step 3's camera run:

- `lost 0`, `crc 0`, `pattern 0` (not counting the few boot-time errors and Step 5)
- `uart ovr 0`, `drop 0`
- camera `crc 0`, `cut 0`, frames `ok` climbing, `i` shows a picture that reacts to the scene
- the ESP32 side (`ESP32 got …` on the MCU console) shows `bad 0 lost 0`

Send back `run1.txt` (and a photo of the wiring if anything failed).

---

## 7. Troubleshooting

| what you see | most likely cause | fix |
|---|---|---|
| `nothing received yet` | ESP32 TX not reaching J8 pin 9, no shared GND, or the ESP32 is not running | check IO14 → J8 **pin 9**, GND ↔ GND, ESP32 Serial Monitor shows it sending |
| `bytes arrived but no good packet`, `frm` climbing | baud mismatch, or no shared GND | `LINK_BAUD` must be the same in `LinkTest.ino` and `nxp/board/cm33_core0/app.h`; tie the grounds |
| `crc` / `frm` climbing only in flood or camera mode | long or loose wires, noise | shorter wires, twist each signal wire with a GND wire; or set **both** `LINK_BAUD` to `115200` (the MCU side then needs a rebuild, section 9) |
| MCU sees the ESP32, but the ESP32 says `no heartbeat from the MCU` | the MCU → ESP32 wire | J8 **pin 12** → ESP32 **IO13** |
| `ESP32 says: camera init FAILED 0x…` | camera ribbon not seated, or weak 5 V | reseat the ribbon (lift the black latch, push the cable fully in, close the latch); power from a solid 5 V |
| `reboots` climbing, `LinkTest up` repeating | ESP32 browns out | separate 5 V / 2 A supply, grounds tied |
| ESP32 will not boot once wired | a wire is on **IO12** (or IO0 / IO2) | the link uses IO13 and IO14 only |
| upload: `Failed to connect to ESP32` | not in download mode | MB: hold IO0, tap RST, release. Adapter: IO0 → GND jumper, RST |
| ESP32 runs the old program / looks dead after upload | IO0 still grounded | remove the jumper, press RST |
| `pyocd list` shows nothing | charge-only cable, or the wrong USB port | data cable, MCU-Link port (J17) |
| `Target type mcxn947vdft not recognized` | the trailing T | `-t mcxn947vdf` |
| `UnicodeEncodeError` from pyOCD | Windows console encoding | `$env:PYTHONIOENCODING = "utf-8"` |
| `uart ovr` or `drop` above 0 | the MCU fell behind (not expected) | send the log, this is a firmware bug |

### Splitting a dead link in half (needs one USB-TTL adapter)

`tools/link_tool.py` can play either board through a USB-TTL adapter, so you can tell which side
is broken. Use an adapter that does 921600 baud (CH340, CP2102 and FTDI all do).

```powershell
python tools/link_tool.py ports      # find the adapter's COM port
```

- **Is the MCX N947 side OK?** Adapter TX → J8 pin 9, adapter RX ← J8 pin 12, GND → GND.
  `python tools/link_tool.py fake-esp32 --port COM5`. The MCU console should behave exactly as in
  section 6 (camera mode sends a moving test pattern instead of a photo).
- **Is the ESP32 side OK?** Adapter RX ← ESP32 IO14, adapter TX → ESP32 IO13, GND → GND.
  `python tools/link_tool.py fake-mcu --port COM5`, then keys `t f c x i` as on the real MCU.

---

## 8. The protocol, briefly

Full definition with comments: [`esp32cam/LinkTest/rover_link.h`](esp32cam/LinkTest/rover_link.h).

```
COBS( type:u8 | seq:u16le | payload: 0..240 bytes | crc16:u16le )  0x00
```

- **COBS** rewrites each packet so it contains no `0x00`; a `0x00` on the wire therefore always
  means "packet ends here". A receiver that starts listening mid-packet, or loses a byte, discards
  only that one packet and is back in step at the next `0x00`.
- **CRC-16/CCITT-FALSE** over type, seq and payload catches flipped bits.
- **seq** counts packets per sender, so the receiver counts packets that never arrived (`lost`),
  and spots a sender reboot (`reboots`) when it jumps back to 0.
- **Heartbeat** 5 times a second both ways; silence for 1 s means the other side is gone.

Packet types `0x01-0x05` are the rover's real messages (TELEMETRY, EVENT, SKILL_CMD,
SKILL_STATUS, HEARTBEAT). `0x10-0x14` exist only for this test.

Checked on a PC before any hardware (`test/test_link.c`): COBS round trips at every length 0-1100
including 254-byte zero-free runs; 20,000 packets with 206 flipped bits and 40 junk bursts
spliced in, where every undamaged packet was recovered in order and every damaged one counted
exactly. `tools/link_tool.py selftest` checks that the Python copy encodes byte-for-byte like the C.

```bash
gcc -O2 -std=c99 -Wall -I esp32cam/LinkTest test/test_link.c -o test_link && ./test_link
./test_link --dump stream.bin && python tools/link_tool.py selftest stream.bin
```

---

## 9. Building the MCX N947 firmware from source (only if you change it)

The prebuilt `.elf` is enough to run the test. Rebuild only after editing `nxp/` or `rover_link.h`.

1. Install the Arm GNU Toolchain (arm-none-eabi, 14.2.Rel1), CMake ≥ 3.30, Ninja, and
   `pip install west`. Set an environment variable `ARMGCC_DIR` to the toolchain folder
   (the one containing `bin/arm-none-eabi-gcc`). Open a new terminal afterwards.
2. Fetch the MCUXpresso SDK (about 2.3 GB, 30+ minutes):

   ```powershell
   west init -m https://github.com/nxp-mcuxpresso/mcuxsdk-manifests.git --mr v26.06.00-LTS mcuxsdk-ws
   cd mcuxsdk-ws
   west update --narrow -o=--depth=1
   ```

3. Copy this firmware into it and build. **`-Dcore_id=cm33_core0` is mandatory**: the MCX N947 has
   two cores and CMake stops with "Please specify core_id for multicore device" without it.

   ```powershell
   python <path-to>/rover-esp32-link/nxp/install.py --sdk <path-to>/mcuxsdk-ws/mcuxsdk
   west build -b frdmmcxn947 mcuxsdk/examples/demo_apps/esp_link -d build_out -p always -- -Dcore_id=cm33_core0
   pyocd flash -t mcxn947vdf build_out/esp_link_cm33_core0.elf
   pyocd reset -t mcxn947vdf
   ```

`west flash` does not work with pyOCD here (the SDK's board only defines the J-Link and LinkServer
runners), so flashing stays a separate pyOCD step.

---

## Linux / macOS notes

- Python tools are the same. Serial ports are `/dev/ttyACM0` (MCU-Link) and `/dev/ttyUSB0`
  (CH340/CP2102) on Linux, `/dev/cu.usbmodem*` / `/dev/cu.usbserial*` on macOS.
- Linux: add yourself to the `dialout` group (`sudo usermod -aG dialout $USER`, log out and in) and
  install pyOCD's udev rules (pyOCD docs, "udev rules") or `pyocd list` will not see the probe.
- `PYTHONIOENCODING` is only needed on Windows.

## Pin choice, for the record

The link uses LPUART0 on P0_12 / P0_13: the pair NXP's own SDK uses for UART examples on this
board, on a header the rover does not use. It clashes with none of the blind-rover firmware's
pins (encoders P0_28/29/31/19, L298N P3_18/19 + P4_2/3, LEDs, SW2/SW3), so the link can move into
the rover firmware unchanged. The Arduino-header UART (LPUART2, J1 pins 2/4 = P4_3/P4_2) was not
used because the rover drives the L298N direction inputs from those two pins.
