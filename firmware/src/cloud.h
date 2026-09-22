#pragma once
#include <Arduino.h>
#include "store.h"

namespace Cloud {

// Join the LAN with the stored creds. Blocks up to ~20s. Returns true once connected.
bool connectWifi(const Creds& c);
bool wifiConnected();

// Ensure we have a registered world session + device. Uses the cached ids if present; otherwise
// calls POST /worlds + POST /sessions/:id/connect-device (kind="robot") with the account API key,
// persisting the result. Mutates c.sessionId/c.deviceId. Returns false on auth/API failure.
bool ensureRegistered(Creds& c);

// Outcome of a control poll — lets the loop tell "retry later" (transient) apart from "stop and
// surface an error" (session deleted, or the API key is bad), exactly like the camera's PostResult.
struct PollResult {
  bool ok = false;          // 2xx with a parseable body
  bool gone = false;        // 404 — session/device deleted server-side → clear cache + re-register
  bool authFailed = false;  // 401/403 — bad/revoked/expired API key → re-provisioning required
  int code = 0;             // raw HTTP status, or <0 on a transport error (no response)
};

// One pending control action the server wants the arm to perform. The arm has no media stream to
// piggyback on, so it polls the /control channel; the body carries the latest queued action(s).
struct Action {
  bool valid = false;            // true once a well-formed action was parsed
  String name;                   // joint|pose|gripper|home|reset|stop|move|back|turn
  // Raw payload JSON for the action (e.g. {"name":"elbow_flex","value":0.3}); parsed by the caller.
  String payloadJson;
};

// GET the /control channel for the next robot action. Fills *res (ok/gone/authFailed/code). When a
// body parses, fills *act (act->valid true if an action was present). Returns res.ok. A 2xx with no
// action is still ok (act->valid stays false) — the arm just holds its pose.
bool pollControl(const Creds& c, PollResult* res, Action* act);

}  // namespace Cloud
