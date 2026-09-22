# SO-101 robot firmware — notes

## Two architectures

**A. Computer-proxy (tethered) — DONE, public.**
Laptop runs a Python script plugged into the SO-101's Feetech USB bus-servo adapter;
the script uses the CommandAGI SDK (`register_robot` → `bridge.run`) and writes joint
goals straight to the bus. No microcontroller. This is the standard LeRobot path and is
the public example repo `commandagi-example-so101` (`so101_arm.py`).

**B. Standalone Wi-Fi (embedded) — BUILT (this firmware).**
The ESP32 itself is the brain: pairs over BLE, joins Wi-Fi, self-registers with CommandAGI
as a `byo.robot`, polls `/control` for actions, and drives the servo bus. No laptop. USB is
power + flashing + an optional dev/debug fallback only. This mirrors exactly how our
ESP32-CAM camera firmware (`commandagi-esp32-cam`) works — and **reuses its code**.

The original USB-serial-only `main.cpp` is now folded into (B) as the dev/debug fallback:
`main.cpp` orchestrates the cloud path and still services the line-delimited JSON protocol
over USB, so the board stays benchtest-able while it runs standalone.

### How (B) is wired — reused from the camera firmware

- **BLE provisioning + crypto + NVS** (`ble_prov.*`, `crypto.*`, `store.*`, `status.*`):
  copied from `commandagi-esp32-cam` and adapted. SAME GATT service (`c0a1d61c-0001-…`) and
  characteristics — the only behavioral change is `INFO.kind = "arm"` /
  `model = "CommandAGI Robot Arm"` (no `mic` field). PIN-sealed PROVISION
  (HKDF-SHA256 → AES-256-GCM) is identical, so the same app seals payloads for both
  devices. Wire contract: `packages/domain/core/src/esp32cam.ts`.
- **Wi-Fi + registration** (`cloud.cpp`): `connectWifi()` then `POST /worlds` →
  `{sessionId}`, `POST /sessions/:id/connect-device` with **`kind:"robot"`** → `{deviceId}`,
  cached in NVS (idempotent; re-registers on 404). The BLE `INFO.kind` is `"arm"` (a
  hardware class); the session/device `kind` is `"robot"` (the world).
- **Control** (`cloud.cpp` `pollControl()` + `main.cpp` `runAction()`): GETs
  `apiBaseUrl + /v1/streams/:sessionId/:deviceId/control` (~4 Hz, `CAGI_CONTROL_POLL_MS`)
  and parses one action. Vocabulary: `joint/pose/gripper/home/reset/stop/move/back/turn`.
  Joints map by name to servo IDs 1–6, normalized values are scaled across each joint's
  `[min,max]`, then sync-written via `sts_bus.h` (already flash-verified). Boots
  de-energized; first motion command energizes.
- **Target**: built on an **ESP32-CAM** module (`board = esp32cam`, PSRAM), so the bus
  runs on the exposed GPIO13/14 and the arm can LATER stream its own POV. `partitions.csv`
  matches the camera's dual-OTA 1.9 MB-app layout (BLE + Wi-Fi + TLS overflows the default
  table). Onboard camera POV is left as a marked `// TODO` in `main.cpp::setup()` — NOT
  implemented now; control-only.
- **Camera/view caveat**: `byo.robot` expects a view. Options unchanged:
  (1) this arm + a separate `byo.camera` (our ESP32-CAM) watching it — cleanest today;
  (2) cameraless to start; (3) flip on the ESP32-CAM's own POV via the TODO above (camera
  uses most GPIO; UART is already on the free GPIO13/14, so it's feasible later).

### Hardware testing still needed (build is clean; not yet flash-tested)

`pio run` builds clean (env `esp32cam`, RAM 17.4%, Flash 63.0% of the 1.9 MB OTA slot), but
the standalone path hasn't run on hardware (the ESP32-CAM wasn't on its dock). Verify on
device: (1) BLE advertise + INFO `kind:"arm"` shows in the app picker; (2) PIN-sealed
provisioning persists + survives reboot; (3) Wi-Fi join + register as `byo.robot`;
(4) `/control` actions move the right joints with correct normalization/clamping;
(5) torque-OFF on boot + on `stop`; (6) BLE+Wi-Fi radio coexistence is stable (modem-sleep
ON, as in `cloud.cpp`).

## Wiring (ESP32 ↔ SO-101 driver, for the standalone build)

Only three logic wires; the driver board's bus transceiver handles the half-duplex bus:

- ESP32 **TX → driver RX**
- ESP32 **RX → driver TX**
- ESP32 **GND → driver GND** (shared ground mandatory)

Current firmware uses UART2 on **GPIO13 (TX → driver RX) / GPIO14 (RX ← driver TX)** at
**1,000,000 baud** (set in `src/config.h`: `CAGI_BUS_TX_PIN` / `CAGI_BUS_RX_PIN`). These are
the exposed, boot-safe pins on the ESP32-CAM module — GPIO16/17 are PSRAM there. The ESP32
can route UART to almost any pins; change to suit the board.

**Servo power is separate.** STS3215 run at ~6–7.4 V (or 12 V variant) at several amps,
from the driver board's own power input — **never** from the ESP32's 5 V/USB. USB powers
only the ESP32 logic. Some driver boards can back-feed regulated 5 V to their header (then
the ESP32 can run off the board); check the silkscreen.

### Hardware identified so far (dev bench, 2026-06-16)

- The arm's wireless ESP32: an **ESP32-CAM** module (classic ESP32 + PSRAM). Build target
  `board = esp32cam` (matches the camera firmware: NimBLE + Wi-Fi + dual-OTA partitions).
  Flashed via USB-TTL adapter (no onboard CDC). (A XIAO ESP32-S3/C3 would need a different
  target + pins.) The earlier `esp32dev` USB-only build is superseded by the standalone one.
- The SO-101 USB driver: **QinHeng CH343** (`1a86:55d3`) → `/dev/ttyACM0`, CDC-ACM. It's a
  transparent USB↔TTL bridge fronting the Feetech bus. The standalone ESP32 bypasses it and
  drives the bus header directly.
- STILL NEEDED for exact pin map: the SO-101 driver board's model / silkscreen photo, and
  confirmation of the wireless ESP32 board model.

## Flashing (ESP32-CAM)

- **Canonical path: the monorepo general flasher** — `make flash-device PRODUCT=arm` (repo root),
  i.e. `scripts/flash-device.mjs --product arm`. It builds this `firmware/` project (env `esp32cam`),
  compiles a per-unit provisioning PIN in (`-DCAGI_PROV_PIN`), flashes over USB, reads the
  `CommandAGI Arm XXXX` BLE suffix back over serial, and appends the unit to `labels.csv`
  (`LOOP=1` for batch). Same tool that flashes the camera — see the repo `scripts/README.md`.
- Manual: `pio run -t upload && pio device monitor` from `firmware/` (env `esp32cam`). The ESP32-CAM
  has no USB-UART of its own — flash via an FTDI/USB-TTL adapter (GPIO0→GND to enter
  download mode, release after the first flash) or the carrier board's bridge.
- **PlatformIO + Python 3.14 gotcha:** unpinned `platform = espressif32` resolves to a
  src/pioarduino build whose Arduino core fails to install (`FRAMEWORK_DIR is None`). Pin
  `platform = espressif32@6.9.0` (already pinned in `platformio.ini`).
- Re-flashing is always safe: no Secure Boot / Flash Encryption, dual-OTA partitions, and
  runtime UART use never blocks flashing. Manual fallback: hold BOOT (GPIO0→GND) on reset.
- Power the ESP32-CAM via the **5V pin** (not 3.3V — see the brownout note below); the
  servo bus runs on the exposed, boot-safe **GPIO13 (→ driver RX) / GPIO14 (← driver TX)**
  at **1,000,000 baud** (GPIO16/17 are PSRAM on this module).

## Safety

- Firmware boots **de-energized** — torque stays OFF until the host sends an explicit
  `goal`/`home`/`torque` command. Resets and re-flashes never jerk a wired arm.
- Per-joint limits in `main.cpp` (`JOINT_MIN/MAX/HOME`) are conservative defaults. **Run
  the LeRobot SO-101 calibration and tighten them before trusting the arm under torque.**

## SO-101 joints

IDs 1..6: `shoulder_pan(1) shoulder_lift(2) elbow_flex(3) wrist_flex(4) wrist_roll(5)
gripper(6)`. STS3215, position 0..4095 over 360°, center 2048, little-endian registers
(Torque_Enable 0x28, Goal_Position 0x2A, Goal_Speed 0x2E, Present_Position 0x38).

## ESP hardware convention (most of our ESPs)

Module-on-board-on-carrier: an ESP-32S Wi-Fi+BT SoC module (metal-can FCC/CE sub-PCB)
soldered onto a Seeed board exposing 3.3V/5V/GND pins, whose male pins seat into female
headers on a USB "dev board" carrier. Power the ESP32-CAM via the **5V pin** (→ onboard
AMS1117-3.3 LDO), **not** the 3.3V pin (bypasses the regulator → brownouts). ESP32-CAM is
brownout-prone (>300 mA Wi-Fi/camera spikes): add a 470–1000 µF bulk cap, share ground,
never feed 5V and 3.3V simultaneously, and never tie two 5 V sources together (back-feed
= the real way to damage hardware).
