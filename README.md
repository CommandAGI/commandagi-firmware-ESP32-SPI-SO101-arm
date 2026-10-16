# ESP32-SPI-SO101-arm

Part of [CommandAGI](https://commandagi.com): connecting agents to real computers, robots and
physical environments. This repository can be cloned independently of the private platform code.

```sh
git clone https://github.com/CommandAGI/commandagi-firmware-ESP32-SPI-SO101-arm.git
cd commandagi-firmware-ESP32-SPI-SO101-arm
```

## Hardware status

Despite the repository's requested `SPI` label, the current servo driver speaks Feetech's
half-duplex **TTL UART** protocol at 1 Mbps (`firmware/src/sts_bus.h`), not SPI. Use the documented
UART wiring; do not infer the electrical interface from the repo name. The default PlatformIO
board is `esp32cam`. Hardware motion and live platform interoperability need supervised validation.


ESP32-based robot control firmware for putting a physical **SO-101 (LeRobot) arm** on
[CommandAGI](https://commandagi.com), plus design notes and hardware references.

This is the **embedded / standalone** track. The public, computer-proxy version (a plain
Python script that runs on a laptop plugged into the SO-101 USB driver) lives in the
public example repo `commandagi-example-connect-a-so101-arm`. This repo is where the on-device firmware
and the **no-computer, Wi-Fi** robot live.

## Standalone (no laptop) — exactly like the CommandAGI cameras

The firmware is now **standalone**: pair over BLE → join Wi-Fi → self-register as a
`byo.robot` under your account → drive the servos from cloud `/control` actions. USB is
power + flashing + an optional dev/debug fallback only. It reuses the **shared device
provisioning contract** (`packages/domain/core/src/esp32cam.ts`) the cameras use — the SAME BLE
GATT service (`c0a1d61c-0001-…`) and INFO/STATUS/PROVISION/COMMAND characteristics — and
reports **`INFO.kind = "arm"`** so the app labels + filters it as a robot arm.

Lifecycle (the app watches STATUS notifications): `idle` → `wifi_connecting` →
`registering` → `streaming` (live). Secrets are PIN-sealed (HKDF-SHA256 → AES-256-GCM) so
a BLE sniffer without the PIN learns nothing. All creds live in NVS, so it reconnects on
its own after a power cycle; a factory-reset wipes only the creds (never the firmware).

### Flow

1. **BLE pairing.** A factory-fresh arm advertises `CommandAGI Arm <MAC4>`. The app reads
   INFO (`kind:"arm"`, `model:"CommandAGI Robot Arm"`, `hwid`, `fw`, `provisioned`,
   `secure`, `salt`), PIN-seals + writes Wi-Fi creds + a per-device `cagi_` API key to
   PROVISION, and subscribes to STATUS.
2. **Self-registration.** The arm joins Wi-Fi, then `POST /worlds` → `{sessionId}` and
   `POST /sessions/:id/connect-device` with `kind:"robot"` → `{deviceId}` (idempotent via
   the NVS-cached ids). It re-registers automatically if those ids later 404.
3. **Control.** It polls `GET /v1/streams/:sessionId/:deviceId/control` (~4 Hz) and drives
   the 6 STS3215 servos from the action it parses.

### Control vocabulary (parsed from the `/control` body)

| verb                     | payload                                   | effect                                       |
| ------------------------ | ----------------------------------------- | -------------------------------------------- |
| `joint`                  | `{"name":"elbow_flex","value":-1..1}`     | one joint, normalized across its `[min,max]` |
| `pose`                   | `{"pose":{"elbow_flex":0.3,"gripper":1}}` | many joints at once (normalized)             |
| `gripper`                | `{"value":0..1}`                          | `0` = closed/min, `1` = open/max             |
| `home` / `reset`         | —                                         | go to the safe tucked home pose              |
| `stop`                   | —                                         | de-energize (torque OFF) immediately         |
| `move` / `back` / `turn` | `{"value":-1..1}?`                        | coarse base-joint nudge (≤25% of range/step) |

Joints (servo IDs 1–6): `shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll
gripper`. Goals are ALWAYS clamped into each joint's `[min,max]`. The arm boots
**de-energized** — torque stays OFF until the first motion command.

## Layout

```
firmware/            PlatformIO project — standalone BLE + Wi-Fi + servo-bus firmware (ESP32-CAM)
  platformio.ini     env:esp32cam — NimBLE + ArduinoJson, dual-OTA partitions (BLE+Wi-Fi+TLS)
  partitions.csv     4MB dual-OTA, no Secure Boot — always re-flashable over USB
  src/main.cpp       orchestrator: BLE → Wi-Fi → register → /control servo drive (+ USB-serial dev)
  src/ble_prov.*     shared CommandAGI BLE provisioning service, reports kind="arm"
  src/cloud.*        Wi-Fi join + register (kind="robot") + /control poll
  src/crypto.*       PIN-keyed AEAD (HKDF-SHA256 → AES-256-GCM) — mirrors core esp32cam.ts
  src/store.*        NVS cred + session/device storage
  src/status.*       STATUS lifecycle (idle→wifi_connecting→registering→streaming)
  src/config.h       UUIDs, kind, pins, PIN, polling cadence
  src/sts_bus.h      Feetech STS protocol (packet framing, sync-write, read)
host-bridge/         Reference: the tethered Python bridge (laptop ↔ ESP32 over serial)
  so101_serial_bridge.py
NOTES.md             Architecture, wiring, hardware, calibration, safety
```

The channels this firmware gives are declared once, in CommandAGI's cloud seed:
`seed/drive/platform/user-commandagi/hardware/ESP32-SPI-SO101-arm/firmware/firmware.tsx`
(`cloud://home/user-commandagi/hardware/ESP32-SPI-SO101-arm/firmware`). That folder's `platformio.ini` builds this
source for the product (env `commandagi-arm`), and the device definition `commandagi-arm` names it. When a change
here changes a channel, change that file in the same step.

## USB-serial dev/debug fallback (kept for bench testing)

The board stays flashable + testable over USB. Line-delimited JSON @ 115200:
`{"t":"goal","pos":[p1..p6]}`, `{"t":"torque","on":bool}`, `{"t":"home"}`, `{"t":"ping"}`;
the firmware replies `{"t":"ready",...}` / `{"t":"state",...}` / `{"t":"ack"}` / `{"t":"err"}`.
A motion command here energizes the arm just like a cloud action.

## Status

- `firmware/` **builds clean** (`pio run`, env `esp32cam`): RAM 17.4%, Flash 63.0% of the
  1.9 MB OTA app slot. The servo bus + the `ready`/`state` JSON dev protocol are
  **flash-verified on real hardware** (Feetech STS3215, 1 Mbps).
- The standalone BLE + Wi-Fi + `/control` path is **built but not yet flash-tested** — the
  ESP32-CAM wasn't on its dock at build time. Needs an on-hardware pairing + register +
  teleop pass (see NOTES.md → "Hardware testing still needed").

See `NOTES.md` for everything.

## License

[MIT](LICENSE).

## Build verification (2026-09-22)

All PlatformIO environments declared by this repository compiled successfully using PlatformIO
6.2.0. This verifies compilation, not physical wiring, sensor operation or live cloud connectivity.
