// The SO-101 arm firmware: what it gives (CommandAGI docs/devices.md § firmware). A device definition that names this
// folder as its firmware is held to these channels by `commandagi check <definition.tsx>`.
//
// arm: the arm's one channel. The firmware polls GET /v1/streams/<session>/<device>/control (src/cloud.cpp,
// CAGI_CONTROL_POLL_MS) and runs the action it gets (src/main.cpp runAction): pose, gripper, home, reset, stop, move,
// back and turn. Its polls are what comes back: the host records each action the arm takes as an ack, and a gap in
// the polls as state unknown (CommandAGI's host-relay driver). The firmware reports no joint positions on this path.
// So the channel is duplex, format robot-ctrl, transport relay.
//
// Not channels: BLE provisioning (how the unit joins), and the USB-serial bench protocol of src/main.cpp (goal,
// torque, home, ping; ready, state, ack, err), which no format or driver reads.
//
// Revision A: the first hardware, an AI-Thinker ESP32-CAM on the SO-101's Feetech bus (env esp32cam).
export default () => (
  <firmware name="CommandAGI SO-101 arm">
    <supports revision="A" />
    <channel id="arm" dir="duplex" medium="records" format="robot-ctrl" transport="relay" />
  </firmware>
);
