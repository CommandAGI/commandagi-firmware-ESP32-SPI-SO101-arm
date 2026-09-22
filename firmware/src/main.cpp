// CommandAGI SO-101 robot-arm firmware — STANDALONE (no laptop)
// ─────────────────────────────────────────────────────────────
// A factory-fresh arm advertises the SHARED CommandAGI BLE provisioning service (the same one the
// camera uses — service c0a1d61c-0001-…, INFO/STATUS/PROVISION/COMMAND chars) reporting INFO.kind="arm".
// The CommandAGI app discovers it, PIN-seals + writes Wi-Fi creds + a per-device account API key, and
// the arm then joins Wi-Fi, self-registers a world session under that account as a byo.robot
// (POST /worlds → POST /sessions/:id/connect-device with kind="robot"), and drives the SO-101's 6x
// Feetech STS3215 servos from control actions it polls off the cloud /control channel. All state lives
// in NVS, so it reconnects on its own after a power cycle. Re-flashable over USB forever (no Secure
// Boot / Flash Encryption); a factory-reset only wipes the creds. See README.md and NOTES.md.
//
// SAFETY: the arm boots DE-ENERGIZED. Torque stays OFF until the first motion command (from the cloud
// OR the USB-serial dev protocol) — so resets and re-flashes never jerk a wired, powered arm.
//
// Servo bus: UART2 remapped to the exposed, boot-safe GPIO13 (→ driver RX) / GPIO14 (← driver TX) at
// 1,000,000 baud (GPIO16/17 are PSRAM on the ESP32-CAM module this is built on). See src/config.h.
//
// ── Cloud control vocabulary (parsed from the /control body) ──────────────────────────────────────
//   joint   {"name":"elbow_flex","value":-1..1}     // one joint, normalized across its [min,max]
//   pose    {"pose":{"elbow_flex":0.3,"gripper":1}} // many joints at once (normalized values)
//   gripper {"value":0..1}                            // 0 = closed/min, 1 = open/max
//   home / reset                                      // go to the safe tucked home pose
//   stop                                              // de-energize (torque OFF) immediately
//   move / back / turn  {"value":-1..1?}              // coarse defaults: nudge base joints
//
// ── USB-serial DEV/debug fallback (line-delimited JSON, 115200) — kept so the board stays testable ─
//   {"t":"goal","pos":[p1..p6]}  {"t":"torque","on":bool}  {"t":"home"}  {"t":"ping"}
//   ESP32→host: {"t":"ready","servos":[...]} / {"t":"state","pos":[...]} / {"t":"ack"} / {"t":"err"}
//
// MIT licensed — see LICENSE.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "sts_bus.h"
#include "config.h"
#include "store.h"
#include "status.h"
#include "cloud.h"
#include "ble_prov.h"

// ---- SO-101 follower joints at servo IDs 1..6 ----
// shoulder_pan(1) shoulder_lift(2) elbow_flex(3) wrist_flex(4) wrist_roll(5) gripper(6)
static const uint8_t NUM_JOINTS = 6;
static const uint8_t SERVO_IDS[NUM_JOINTS] = {1, 2, 3, 4, 5, 6};
static const char* JOINT_NAMES[NUM_JOINTS] = {
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
static const uint8_t GRIPPER_IDX = 5;  // index of the gripper joint in the arrays above

// Per-joint safe limits in servo units (0..4095). Conservative defaults — run the LeRobot
// calibration for your build and tighten these before trusting them under torque. Goals are ALWAYS
// clamped into [min,max] on this firmware.
static const uint16_t JOINT_MIN[NUM_JOINTS]  = { 700,  700,  700,  700,  100,  1700};
static const uint16_t JOINT_MAX[NUM_JOINTS]  = {3400, 3400, 3400, 3400, 3995, 2700};
// Rest / home pose (a safe, low-torque tucked posture).
static const uint16_t JOINT_HOME[NUM_JOINTS] = {2048, 2400, 2048, 2048, 2048, 2400};
// Goal move speed (0 = max). Modest speed keeps motion smooth + safe.
static const uint16_t MOVE_SPEED = 800;

HardwareSerial BusSerial(2);
STSBus bus(BusSerial);

uint16_t goalPos[NUM_JOINTS];
bool torqueOn = false;

// ── Servo helpers (shared by the cloud path AND the USB-serial dev path) ──────────────────────────

static uint16_t clampJoint(uint8_t i, long v) {
  if (v < JOINT_MIN[i]) v = JOINT_MIN[i];
  if (v > JOINT_MAX[i]) v = JOINT_MAX[i];
  return (uint16_t)v;
}

// Map a normalized value across a joint's [min,max]. For a regular joint `n` is -1..1 (-1 = min,
// 0 = center, +1 = max); for the gripper we feed 0..1 (0 = min/closed, 1 = max/open).
static uint16_t normToServo(uint8_t i, float n, float lo, float hi) {
  if (n < lo) n = lo;
  if (n > hi) n = hi;
  const float t = (n - lo) / (hi - lo);  // 0..1 across the input range
  const float u = JOINT_MIN[i] + t * (float)(JOINT_MAX[i] - JOINT_MIN[i]);
  return clampJoint(i, (long)(u + 0.5f));
}

static int jointIndexByName(const char* name) {
  if (!name) return -1;
  for (uint8_t i = 0; i < NUM_JOINTS; i++)
    if (strcmp(name, JOINT_NAMES[i]) == 0) return (int)i;
  return -1;
}

static void setTorque(bool on) {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) bus.torqueEnable(SERVO_IDS[i], on);
  torqueOn = on;
}

static void writeGoals() {
  if (!torqueOn) setTorque(true);  // first motion energizes the arm (boots de-energized; see header)
  bus.syncWritePositions(SERVO_IDS, goalPos, NUM_JOINTS);
  // SYNC_WRITE sets positions; ensure a sane move speed per servo.
  for (uint8_t i = 0; i < NUM_JOINTS; i++) bus.writeWord(SERVO_IDS[i], STS_GOAL_SPEED, MOVE_SPEED);
}

static void goHome() {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) goalPos[i] = JOINT_HOME[i];
  writeGoals();
}

// ── Cloud action dispatch — drive the bus from a parsed Cloud::Action ─────────────────────────────

static void applyJoint(int idx, float value, bool isGripper) {
  if (idx < 0 || idx >= NUM_JOINTS) return;
  goalPos[idx] = isGripper ? normToServo(idx, value, 0.0f, 1.0f)   // gripper: 0..1 (closed→open)
                           : normToServo(idx, value, -1.0f, 1.0f); // joint: -1..1 across [min,max]
  writeGoals();
}

// Run one control action against the servo bus. `name` is the verb; `payloadJson` is that action's
// whole JSON object (so we can pull verb-specific fields). Unknown verbs are ignored (forward-compat).
static void runAction(const String& name, const String& payloadJson) {
  JsonDocument d;
  if (deserializeJson(d, payloadJson)) return;

  if (name == "stop") {
    setTorque(false);  // de-energize immediately — the strongest safety action
    Serial.println("[act] stop → torque OFF");
    return;
  }
  if (name == "home" || name == "reset") {
    goHome();
    Serial.println("[act] home");
    return;
  }
  if (name == "joint") {
    const char* jn = d["name"] | "";
    int idx = jointIndexByName(jn);
    if (idx < 0) { Serial.printf("[act] joint: unknown '%s'\n", jn); return; }
    applyJoint(idx, d["value"] | 0.0f, idx == GRIPPER_IDX);
    Serial.printf("[act] joint %s = %.2f → %u\n", jn, (double)(d["value"] | 0.0f), goalPos[idx]);
    return;
  }
  if (name == "gripper") {
    applyJoint(GRIPPER_IDX, d["value"] | 0.0f, /*isGripper=*/true);
    Serial.printf("[act] gripper = %.2f → %u\n", (double)(d["value"] | 0.0f), goalPos[GRIPPER_IDX]);
    return;
  }
  if (name == "pose") {
    // {"pose":{"elbow_flex":0.3,"gripper":1,...}} — set every named joint, then one sync write.
    JsonObject pose = d["pose"].as<JsonObject>();
    if (pose.isNull()) { Serial.println("[act] pose: no map"); return; }
    bool any = false;
    for (JsonPair kv : pose) {
      int idx = jointIndexByName(kv.key().c_str());
      if (idx < 0) continue;
      const float v = kv.value().as<float>();
      goalPos[idx] = (idx == GRIPPER_IDX) ? normToServo(idx, v, 0.0f, 1.0f)
                                          : normToServo(idx, v, -1.0f, 1.0f);
      any = true;
    }
    if (any) { writeGoals(); Serial.println("[act] pose applied"); }
    return;
  }
  // Coarse default verbs (move/back/turn): nudge the base joints by a fraction of their range. These
  // are intentionally simple — fine control comes from joint/pose. `value` (-1..1) scales the nudge.
  if (name == "move" || name == "back" || name == "turn") {
    const float v = d["value"].is<float>() ? d["value"].as<float>() : (name == "back" ? -0.3f : 0.3f);
    const int idx = (name == "turn") ? 0 : 1;  // turn → shoulder_pan, move/back → shoulder_lift
    const long span = (long)(JOINT_MAX[idx] - JOINT_MIN[idx]);
    long target = (long)goalPos[idx] + (long)(v * 0.25f * (float)span);  // ≤25% of the range per step
    goalPos[idx] = clampJoint(idx, target);
    writeGoals();
    Serial.printf("[act] %s → %s=%u\n", name.c_str(), JOINT_NAMES[idx], goalPos[idx]);
    return;
  }
  Serial.printf("[act] ignoring unknown verb '%s'\n", name.c_str());
}

// ── USB-serial DEV/debug fallback (the original tethered protocol — kept for bench testing) ───────

static void sendAck() { Serial.println("{\"t\":\"ack\"}"); }

static void sendErr(const char* msg) {
  JsonDocument d;
  d["t"] = "err";
  d["msg"] = msg;
  serializeJson(d, Serial);
  Serial.println();
}

static void sendReady() {
  JsonDocument d;
  d["t"] = "ready";
  JsonArray arr = d["servos"].to<JsonArray>();
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (bus.ping(SERVO_IDS[i])) arr.add(SERVO_IDS[i]);
  }
  serializeJson(d, Serial);
  Serial.println();
}

static void sendState() {
  JsonDocument d;
  d["t"] = "state";
  JsonArray arr = d["pos"].to<JsonArray>();
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    int p = bus.readPosition(SERVO_IDS[i]);
    arr.add(p);  // -1 if the read failed
  }
  serializeJson(d, Serial);
  Serial.println();
}

static void handleSerialLine(const String& line) {
  JsonDocument d;
  if (deserializeJson(d, line)) { sendErr("bad json"); return; }

  const char* t = d["t"] | "";
  if (strcmp(t, "goal") == 0) {
    JsonArray pos = d["pos"].as<JsonArray>();
    if (pos.isNull() || pos.size() != NUM_JOINTS) { sendErr("goal needs 6 positions"); return; }
    uint8_t i = 0;
    for (JsonVariant v : pos) { goalPos[i] = clampJoint(i, v.as<long>()); i++; }
    writeGoals();
    sendAck();
  } else if (strcmp(t, "torque") == 0) {
    setTorque(d["on"] | false);
    sendAck();
  } else if (strcmp(t, "home") == 0) {
    goHome();
    sendAck();
  } else if (strcmp(t, "ping") == 0) {
    sendReady();
  } else {
    sendErr("unknown command");
  }
}

// ── Orchestration ─────────────────────────────────────────────────────────────────────────────────

namespace {
Creds g_creds;
bool g_live = false;          // registered + actively polling (STATUS "streaming")
uint32_t g_nextControl = 0;
uint32_t g_retryAt = 0;

void reloadCreds() { g_creds = Store::load(); }

// Turn a failed control poll into the right recovery + a VISIBLE status. Returns true if the loop
// should bail this iteration. Mirrors the camera firmware's handlePostFailure().
bool handlePollFailure(const Cloud::PollResult& r) {
  if (r.ok) return false;
  if (r.gone) {  // 404 — session/device deleted server-side: drop the cache and re-register cleanly
    Store::clearRegistration();
    g_creds.sessionId = "";
    g_creds.deviceId = "";
    g_live = false;
    g_retryAt = 0;
    Status::set("registering", "session removed — reconnecting");
    return true;
  }
  if (r.authFailed) {  // 401/403 — the API key itself is rejected: only re-provisioning can fix it
    g_live = false;
    g_retryAt = millis() + 30000;  // stop hammering; surface a clear, app-readable error
    Status::set("register_failed", "API key rejected (" + String(r.code) + ") — re-pair in the app");
    return true;
  }
  // Transient (5xx / timeout / network hiccup): note it once on the live→down edge, then retry soon.
  if (g_live) Status::set("registering", "control poll " + String(r.code));
  g_live = false;
  g_retryAt = millis() + 3000;
  return true;
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n[boot] CommandAGI SO-101 arm " CAGI_FW_VERSION);

  // Servo bus up; do NOT energize. Torque stays OFF until the first motion command (cloud or serial),
  // so a wired+powered arm never jerks across a reset/re-flash. Goals seed to the home pose.
  bus.begin(CAGI_BUS_BAUD, CAGI_BUS_RX_PIN, CAGI_BUS_TX_PIN);
  delay(200);
  for (uint8_t i = 0; i < NUM_JOINTS; i++) goalPos[i] = JOINT_HOME[i];
  torqueOn = false;

  Store::begin();
  Status::set("idle");

  // Bring up BLE FIRST: NimBLE needs internal RAM for its mutexes/queues; initializing other radios
  // first can starve it (a null-mutex assert + boot loop). BLE-first reserves its memory and lets the
  // STATUS notifier be wired before anything else can fail. (Same ordering as the camera firmware.)
  BleProv::begin();

  // TODO: optional onboard camera POV. The arm is an ESP32-CAM, so it could ALSO stream its own view
  // (Camera::begin() + Cloud::postFrame to /v1/streams) exactly like apps/clients/firmware/esp32-cam. Slot the
  // camera bring-up here (see apps/clients/firmware/esp32-cam/src/camera.cpp) and a frame POST in the loop; report
  // it as a second device on the same session. Intentionally NOT implemented now — control-only.

  if (Store::hasCreds()) {
    reloadCreds();
    Serial.println("[boot] have stored creds — connecting");
  } else {
    Serial.println("[boot] factory-fresh — waiting for provisioning over BLE");
  }

  // Announce ourselves on the USB-serial dev channel too (harmless when nothing's listening).
  sendReady();
}

void loop() {
  // 1) USB-serial DEV fallback — always serviced, so the board stays benchtest-able over USB even
  //    while it runs standalone. A motion command here energizes the arm just like a cloud action.
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      buf.trim();
      if (buf.length()) handleSerialLine(buf);
      buf = "";
    } else if (c != '\r') {
      buf += c;
      if (buf.length() > 240) buf = "";  // guard against runaway input
    }
  }

  // 2) Cloud control path. The app wrote fresh creds → reload and force a clean reconnect.
  if (BleProv::consumeNewProvisioning()) {
    reloadCreds();
    g_live = false;
    g_retryAt = 0;
    WiFi.disconnect(true, false);
  }

  if (!Store::hasCreds()) {  // nothing to do but advertise + service serial
    delay(50);
    return;
  }

  uint32_t now = millis();
  if (now < g_retryAt) { delay(20); return; }

  if (!Cloud::wifiConnected()) {
    g_live = false;
    if (!Cloud::connectWifi(g_creds)) {
      g_retryAt = millis() + 5000;  // out of range / bad PSK — back off, app can re-provision
      return;
    }
  }

  if (!Cloud::ensureRegistered(g_creds)) {
    g_retryAt = millis() + 8000;  // bad/revoked API key or API down
    return;
  }

  // 3) Poll the /control channel for the next action and drive the bus.
  if (now >= g_nextControl) {
    Cloud::PollResult res;
    Cloud::Action act;
    Cloud::pollControl(g_creds, &res, &act);
    if (handlePollFailure(res)) return;
    if (!g_live) { Status::set("streaming"); g_live = true; }  // registered + polling = live
    if (act.valid) runAction(act.name, act.payloadJson);
    g_nextControl = millis() + CAGI_CONTROL_POLL_MS;
  }

  delay(10);
}
