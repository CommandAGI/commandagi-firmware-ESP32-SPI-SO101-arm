#include "cloud.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "config.h"
#include "status.h"

// Ported from the camera firmware (apps/clients/firmware/ESP32-32S-OV3660-cam/src/cloud.cpp). Same Wi-Fi join + the same
// POST /worlds → POST /sessions/:id/connect-device registration handshake, but:
//   - connect-device reports kind="robot" (the camera reports kind="camera"), and
//   - instead of POSTing media frames, the arm GETs the /control channel for the next action to run.
// Everything else (reused TLS client, PostResult-style error taxonomy, NVS-cached session/device) is
// identical so the two firmwares stay in lockstep.

namespace {
// One reusable TLS client for all cloud calls (keeps the handshake warm between polls).
WiFiClientSecure g_tls;
bool g_tlsReady = false;

void ensureTls() {
  if (g_tlsReady) return;
  // NOTE: setInsecure() skips server-cert validation — the pragmatic default for a hobby device on
  // your own LAN; it survives Cloudflare cert rotation. The API key still authenticates the device.
  g_tls.setInsecure();
  g_tlsReady = true;
}

// Authenticated JSON POST; returns HTTP status (or <0 on transport error) and fills bodyOut.
int postJson(const String& url, const String& apiKey, const String& bodyIn, String& bodyOut) {
  ensureTls();
  HTTPClient http;
  http.setReuse(true);
  if (!http.begin(g_tls, url)) return -1;
  http.addHeader("Authorization", "Bearer " + apiKey);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t*)bodyIn.c_str(), bodyIn.length());
  bodyOut = (code > 0) ? http.getString() : "";
  http.end();
  return code;
}

// Parse a /control body into one pending Action. The server-side shape for robot control is still
// being finalized; this parser is intentionally permissive and accepts any of:
//   { "action": { "name": "...", ... } }      // single action wrapped under `action`
//   { "name": "...", ... }                     // single action at the top level
//   { "actions": [ { "name": "...", ... } ] }  // a queue → we take the first
// The matched action's whole object is re-serialized into act->payloadJson so the caller (main.cpp)
// can pull out whatever fields that verb needs (value / pose map / etc.). A 2xx with no action is
// valid — act->valid stays false and the arm just holds its pose.
void parseAction(const String& body, Cloud::Action* act) {
  if (!act || body.length() == 0) return;
  JsonDocument doc;
  if (deserializeJson(doc, body)) return;

  JsonVariant a;
  if (doc["action"].is<JsonObject>()) a = doc["action"];
  else if (doc["actions"].is<JsonArray>() && doc["actions"].size() > 0) a = doc["actions"][0];
  else if (doc["name"].is<const char*>()) a = doc.as<JsonVariant>();
  else return;

  const char* name = a["name"] | "";
  if (!name || strlen(name) == 0) return;
  act->name = name;
  serializeJson(a, act->payloadJson);
  act->valid = true;
}
}  // namespace

namespace Cloud {

bool wifiConnected() { return WiFi.status() == WL_CONNECTED; }

bool connectWifi(const Creds& c) {
  Status::set("wifi_connecting");
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  // The classic ESP32 shares ONE radio between Wi-Fi and BLE. The coexistence layer REQUIRES Wi-Fi
  // modem sleep when BLE is also active — disabling it (setSleep(false)) aborts with
  // "Should enable WiFi modem sleep when both WiFi and Bluetooth are enabled". Keep it on.
  WiFi.setSleep(true);
  WiFi.begin(c.ssid.c_str(), c.psk.length() ? c.psk.c_str() : nullptr);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Status::set("wifi_failed", "could not join " + c.ssid);
    return false;
  }
  Status::setNetwork(WiFi.localIP().toString());
  Serial.printf("[wifi] connected: %s\n", WiFi.localIP().toString().c_str());
  return true;
}

bool ensureRegistered(Creds& c) {
  if (c.sessionId.length() && c.deviceId.length()) return true;  // reuse cached registration

  Status::set("registering");
  const String name = c.deviceName.length() ? c.deviceName : String("My robot arm");

  // 1) Create the agentless world session.
  String body;
  {
    JsonDocument d;
    d["title"] = name;
    String req;
    serializeJson(d, req);
    int code = postJson(c.apiBaseUrl + "/worlds", c.apiKey, req, body);
    if (code != 200) {
      Status::set("register_failed", "POST /worlds → " + String(code));
      return false;
    }
    JsonDocument resp;
    if (deserializeJson(resp, body) || !resp["sessionId"].is<const char*>()) {
      Status::set("register_failed", "bad /worlds response");
      return false;
    }
    c.sessionId = String((const char*)resp["sessionId"]);
  }

  // 2) Connect this device as a robot (kind="robot" → byo.robot world). The BLE INFO advertises
  //    kind="arm" (a hardware class) so the app can pick it; the world/session kind is "robot".
  {
    JsonDocument d;
    d["kind"] = "robot";
    d["name"] = name;
    String req;
    serializeJson(d, req);
    int code = postJson(c.apiBaseUrl + "/sessions/" + c.sessionId + "/connect-device", c.apiKey, req, body);
    if (code != 200) {
      Status::set("register_failed", "connect-device → " + String(code));
      c.sessionId = "";
      return false;
    }
    JsonDocument resp;
    if (deserializeJson(resp, body) || !resp["deviceId"].is<const char*>()) {
      Status::set("register_failed", "bad connect-device response");
      c.sessionId = "";
      return false;
    }
    c.deviceId = String((const char*)resp["deviceId"]);
  }

  Store::saveRegistration(c.sessionId, c.deviceId);
  Status::setRegistration(c.sessionId, c.deviceId);
  Serial.printf("[cloud] registered session=%s device=%s\n", c.sessionId.c_str(), c.deviceId.c_str());
  return true;
}

bool pollControl(const Creds& c, PollResult* res, Action* act) {
  PollResult local;
  PollResult* r = res ? res : &local;
  *r = PollResult{};
  if (act) *act = Action{};

  ensureTls();
  String url = c.apiBaseUrl + "/v1/streams/" + c.sessionId + "/" + c.deviceId + "/control";
  HTTPClient http;
  http.setReuse(true);
  http.setTimeout(8000);  // bound the call so a black-hole network can't wedge the loop indefinitely
  if (!http.begin(g_tls, url)) {
    r->code = -1;
    Serial.println("[cloud] control GET: begin() failed (TLS/URL)");
    return false;
  }
  http.addHeader("Authorization", "Bearer " + c.apiKey);
  int code = http.GET();
  String body = (code > 0) ? http.getString() : "";
  http.end();
  r->code = code;
  r->gone = (code == 404);                       // session/device deleted server-side
  r->authFailed = (code == 401 || code == 403);  // bad/revoked/expired API key
  r->ok = code >= 200 && code < 300;
  if (r->ok) parseAction(body, act);
  else Serial.printf("[cloud] control GET → %d\n", code);
  return r->ok;
}

}  // namespace Cloud
