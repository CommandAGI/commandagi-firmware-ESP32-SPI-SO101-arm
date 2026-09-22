#include "ble_prov.h"
#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include "config.h"
#include "store.h"
#include "status.h"
#include "crypto.h"

// Ported from the camera firmware (apps/clients/firmware/esp32-cam/src/ble_prov.cpp). Same GATT service/chars,
// same PIN-sealed PROVISION write, same STATUS notify — the ONLY behavioral change is INFO reports
// kind="arm" + model="CommandAGI Robot Arm" (no mic field) so the app labels this device a robot.

namespace {
NimBLECharacteristic* g_info = nullptr;
NimBLECharacteristic* g_status = nullptr;
volatile bool g_newProvisioning = false;
uint8_t g_salt[16];   // per-boot provisioning salt (published in INFO so the app derives the PIN key)
String g_saltHex;

// Derive the device id from the eFuse MAC (NOT NimBLEDevice::getAddress(), which would pend on the
// BLE host's mutex *before* NimBLEDevice::init() has created it — a null-handle assert + boot loop).
String fullMac() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[13];
  for (int i = 0; i < 6; i++) snprintf(buf + i * 2, 3, "%02X", (uint8_t)((mac >> (8 * i)) & 0xFF));
  return String(buf);
}

String macSuffix() {
  String mac = fullMac();
  return mac.length() >= 4 ? mac.substring(mac.length() - 4) : mac;
}

String infoJson() {
  JsonDocument d;
  d["kind"] = CAGI_DEVICE_KIND;  // "arm" — the app recognizes + labels this as a robot arm
  d["model"] = CAGI_MODEL;
  d["fw"] = CAGI_FW_VERSION;
  d["hwid"] = fullMac();
  d["name"] = String(CAGI_ADV_NAME_PREFIX) + macSuffix();
  d["provisioned"] = Store::hasCreds();
  d["secure"] = (bool)CAGI_PROV_SECURE;
#if CAGI_PROV_SECURE
  d["salt"] = g_saltHex;  // the app derives the PIN key from this + the PIN
#endif
  String out;
  serializeJson(d, out);
  return out;
}

void blinkIdentify() {
#ifdef LED_GPIO_NUM
  pinMode(LED_GPIO_NUM, OUTPUT);
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_GPIO_NUM, HIGH);
    delay(120);
    digitalWrite(LED_GPIO_NUM, LOW);
    delay(120);
  }
#endif
}

// PROVISION write: parse the provisioning JSON, persist, signal the main loop.
class ProvisionCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* ch) override {
    std::string raw = ch->getValue();
    String jsonStr;
#if CAGI_PROV_SECURE
    // Sealed: IV(12) || AES-256-GCM(ct) || tag(16), keyed by the PIN + per-boot salt.
    if (!Crypto::openSealed((const uint8_t*)raw.data(), raw.size(), CAGI_PROV_PIN, g_salt, sizeof(g_salt), jsonStr)) {
      ::Status::set("error", "wrong PIN or corrupt payload");
      return;
    }
#else
    jsonStr = String(raw.c_str());
#endif
    JsonDocument d;
    if (deserializeJson(d, jsonStr)) {
      ::Status::set("error", "bad provisioning JSON");
      return;
    }
    const char* ssid = d["ssid"];
    const char* apiKey = d["apiKey"];
    if (!ssid || !apiKey || strlen(ssid) == 0 || strlen(apiKey) < 8) {
      ::Status::set("error", "provisioning missing ssid/apiKey");
      return;
    }
    Creds c;
    c.ssid = ssid;
    c.psk = d["psk"].is<const char*>() ? (const char*)d["psk"] : "";
    c.apiBaseUrl = d["apiBaseUrl"].is<const char*>() ? (const char*)d["apiBaseUrl"] : "https://api.commandagi.com";
    c.apiKey = apiKey;
    c.deviceName = d["deviceName"].is<const char*>() ? (const char*)d["deviceName"]
                                                     : String(CAGI_ADV_NAME_PREFIX) + macSuffix();
    Store::saveProvisioning(c);
    if (g_info) g_info->setValue(infoJson());  // refresh provisioned=true
    g_newProvisioning = true;
  }
};

// COMMAND write: single token — identify | reboot | factory-reset.
class CommandCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* ch) override {
    String cmd = ch->getValue().c_str();
    cmd.trim();
    if (cmd == "identify") {
      blinkIdentify();
    } else if (cmd == "reboot") {
      delay(200);
      ESP.restart();
    } else if (cmd == "factory-reset") {
      Store::factoryReset();
      delay(200);
      ESP.restart();
    }
  }
};
}  // namespace

namespace BleProv {

String hwSuffix() { return macSuffix(); }
String hwid() { return fullMac(); }

void begin() {
  // Fresh per-boot salt for the PIN key derivation (published in INFO).
  Crypto::randomBytes(g_salt, sizeof(g_salt));
  g_saltHex = Crypto::toHex(g_salt, sizeof(g_salt));

  String advName = String(CAGI_ADV_NAME_PREFIX) + macSuffix();
  NimBLEDevice::init(advName.c_str());
  NimBLEDevice::setMTU(CAGI_CAM_MTU);

#if CAGI_BLE_REQUIRE_BONDING
  // Defense-in-depth: also require an encrypted, passkey-authenticated link (OS prompts for the PIN).
  NimBLEDevice::setSecurityAuth(true, true, true);  // bond, MITM, secure connections
  NimBLEDevice::setSecurityPasskey(CAGI_BLE_PASSKEY);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
  const uint32_t kWriteProps = NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN;
#else
  const uint32_t kWriteProps = NIMBLE_PROPERTY::WRITE;
#endif

  NimBLEServer* server = NimBLEDevice::createServer();
  NimBLEService* svc = server->createService(CAGI_CAM_SERVICE_UUID);

  g_info = svc->createCharacteristic(CAGI_CAM_CHAR_INFO, NIMBLE_PROPERTY::READ);
  g_info->setValue(infoJson());

  g_status = svc->createCharacteristic(CAGI_CAM_CHAR_STATUS, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_status->setValue(::Status::toJson());

  NimBLECharacteristic* prov = svc->createCharacteristic(CAGI_CAM_CHAR_PROVISION, kWriteProps);
  prov->setCallbacks(new ProvisionCb());

  NimBLECharacteristic* cmd = svc->createCharacteristic(CAGI_CAM_CHAR_COMMAND, kWriteProps);
  cmd->setCallbacks(new CommandCb());

  svc->start();

  // Push every status change to subscribers (the app's progress UI).
  ::Status::onChange([](const ::Status::Snapshot&) {
    if (g_status) {
      g_status->setValue(::Status::toJson());
      g_status->notify();
    }
    if (g_info) g_info->setValue(infoJson());
  });

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(CAGI_CAM_SERVICE_UUID);
  adv->setName(advName.c_str());
  adv->setScanResponse(true);
  NimBLEDevice::startAdvertising();
  Serial.printf("[ble] advertising as '%s' (kind=%s)\n", advName.c_str(), CAGI_DEVICE_KIND);
}

bool consumeNewProvisioning() {
  if (!g_newProvisioning) return false;
  g_newProvisioning = false;
  return true;
}

}  // namespace BleProv
