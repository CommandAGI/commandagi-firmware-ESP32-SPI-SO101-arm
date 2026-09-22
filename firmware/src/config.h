#pragma once
// Compile-time configuration for the CommandAGI SO-101 robot-arm firmware.
//
// This is the STANDALONE, no-laptop build: the ESP32 advertises a BLE provisioning service
// (identical wire contract to the CommandAGI camera firmware, but reporting kind="arm"), is handed
// Wi-Fi creds + a per-device account API key over BLE, then self-registers a machine session under
// that account (kind="robot") and drives the SO-101's 6 Feetech STS3215 servos from control actions
// it polls from the cloud. See README.md / NOTES.md and packages/domain/core/src/esp32cam.ts.

// Firmware version reported over BLE (INFO.fw) — bump on each release.
#define CAGI_FW_VERSION "1.0.0"

// Human model string reported over BLE (INFO.model). The app uses INFO.kind="arm" + this to label
// the device as a robot in the picker.
#ifndef CAGI_MODEL
#define CAGI_MODEL "CommandAGI Robot Arm"
#endif

// Device class reported in INFO.kind. The CRITICAL difference from the camera ("camera"): the app
// recognizes + labels this device as a robot arm. The connect-device call below uses kind="robot".
#define CAGI_DEVICE_KIND "arm"

// ── BLE provisioning GATT contract ──────────────────────────────────────────────────────────────
// MUST stay byte-for-byte identical to packages/domain/core/src/esp32cam.ts and the camera firmware — the
// SAME service + characteristic UUIDs, so the existing app BLE client provisions this device too.
// (The names keep the CAM_ prefix only because they're the shared cross-device provisioning UUIDs.)
#define CAGI_CAM_SERVICE_UUID   "c0a1d61c-0001-4a17-9c0a-1d61cab00001"
#define CAGI_CAM_CHAR_INFO      "c0a1d61c-0002-4a17-9c0a-1d61cab00001"  // read       — Info JSON (kind="arm")
#define CAGI_CAM_CHAR_STATUS    "c0a1d61c-0003-4a17-9c0a-1d61cab00001"  // read+notify — Status JSON
#define CAGI_CAM_CHAR_PROVISION "c0a1d61c-0004-4a17-9c0a-1d61cab00001"  // write      — Provisioning JSON
#define CAGI_CAM_CHAR_COMMAND   "c0a1d61c-0005-4a17-9c0a-1d61cab00001"  // write      — identify|reboot|factory-reset
#define CAGI_CAM_MTU            512

// BLE advertised name prefix (a 4-hex MAC suffix is appended, e.g. "CommandAGI Arm A1B2").
#define CAGI_ADV_NAME_PREFIX "CommandAGI Arm "

// ── Provisioning security (PIN-keyed AEAD) ──────────────────────────────────────────────────────
// Identical scheme to the camera: HKDF-SHA256 → AES-256-GCM, keyed by CAGI_PROV_PIN + a per-boot
// salt. A BLE sniffer can't read the Wi-Fi password or account key without the PIN (printed on the
// unit's label). The default below is for the reference build only — CHANGE IT per device.
#define CAGI_PROV_SECURE 1
#ifndef CAGI_PROV_PIN
#define CAGI_PROV_PIN "123456"
#endif
// HKDF `info` — keep in lockstep with CAGI_PROV_HKDF_INFO in packages/domain/core/src/esp32cam.ts. The arm
// shares the camera's provisioning crypto so the same app code seals payloads for both.
#define CAGI_PROV_HKDF_INFO "cagi-cam-prov-v1"

// Optional defense-in-depth: also require BLE link-layer bonding + a passkey (the OS prompts for it).
// Off by default (the app-layer AEAD already protects the secrets); set to 1 to enforce bonding.
#define CAGI_BLE_REQUIRE_BONDING 0
#define CAGI_BLE_PASSKEY 123456

// ── Control polling ─────────────────────────────────────────────────────────────────────────────
// How often (ms) to poll the cloud /control channel for the next robot action. The arm has no media
// stream to piggyback on (unlike the camera), so it polls on this fixed cadence while live. Lower =
// snappier teleop but more TLS/CPU. 250ms (~4Hz) is a sane real-time-ish default.
#define CAGI_CONTROL_POLL_MS 250

// ── NVS (persistent creds) ──────────────────────────────────────────────────────────────────────
#define CAGI_NVS_NAMESPACE "cagi"  // a factory-reset erases exactly this namespace

// ── Servo bus wiring (ESP32-CAM: UART2 remapped to exposed GPIO13/14) ───────────────────────────
// The arm is built on an ESP32-CAM module so it COULD later stream its own POV. GPIO16/17 are NOT
// usable there (PSRAM / not broken out), so the bus uses the exposed, boot-safe GPIO13 + GPIO14.
//   ESP32 GPIO13 -> driver RX,  GPIO14 -> driver TX,  GND -> GND.
#define CAGI_BUS_RX_PIN 14   // <- driver TX
#define CAGI_BUS_TX_PIN 13   // -> driver RX
#define CAGI_BUS_BAUD   1000000

// On-board flash LED — used as an "identify" indicator (AI-Thinker ESP32-CAM).
#ifndef LED_GPIO_NUM
#define LED_GPIO_NUM 4
#endif
