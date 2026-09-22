#pragma once
#include <Arduino.h>

// BLE provisioning service — advertises the SAME CommandAGI provisioning GATT service the camera uses
// (service c0a1d61c-0001-… + identical INFO/STATUS/PROVISION/COMMAND chars) so the mobile/desktop app
// can recognize the device, read INFO, write Wi-Fi creds + account API key (PROVISION), and watch
// STATUS notifications. The ONE difference is INFO.kind = "arm" (the app labels it a robot arm). See
// packages/domain/core/src/esp32cam.ts for the wire contract.
namespace BleProv {
void begin();
// Returns true exactly once after the app writes a fresh, valid provisioning payload (creds are
// already persisted to NVS by then). The main loop polls this to (re)connect.
bool consumeNewProvisioning();
// Short 4-hex uppercase suffix of the device MAC — used in the advertised name + default device name.
String hwSuffix();
String hwid();  // full MAC string
}  // namespace BleProv
