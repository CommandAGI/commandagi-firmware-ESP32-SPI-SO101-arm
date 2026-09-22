// sts_bus.h — minimal Feetech STS3215 serial-bus servo driver (header-only).
//
// The STS3215 servos on the SO-101 / SO-ARM100 arm speak Feetech's half-duplex
// single-wire TTL UART protocol (default 1,000,000 baud). This driver talks to
// them through the SO-ARM bus servo driver board (Waveshare / Feetech), which
// handles the half-duplex direction switching automatically, so we only need a
// plain hardware UART (here ESP32 UART2: TX=GPIO17, RX=GPIO16).
//
// Packet format:
//   0xFF 0xFF  ID  LEN  INST  PARAM...  CHECKSUM
//     LEN      = (num params) + 2
//     CHECKSUM = ~(ID + LEN + INST + sum(params)) & 0xFF
//
// STS series registers are LITTLE-ENDIAN (low byte first). Position is 0..4095
// over 360 degrees, center 2048.
//
// MIT licensed — see LICENSE.

#pragma once

#include <Arduino.h>

// ---- Instructions ----
static const uint8_t STS_INST_PING       = 0x01;
static const uint8_t STS_INST_READ       = 0x02;
static const uint8_t STS_INST_WRITE      = 0x03;
static const uint8_t STS_INST_SYNC_WRITE = 0x83;

// ---- Register addresses (STS series) ----
static const uint8_t STS_TORQUE_ENABLE   = 40;  // 0x28, 1 byte
static const uint8_t STS_GOAL_POSITION   = 42;  // 0x2A, 2 bytes (little-endian)
static const uint8_t STS_GOAL_SPEED      = 46;  // 0x2E, 2 bytes (little-endian)
static const uint8_t STS_PRESENT_POSITION = 56; // 0x38, 2 bytes (little-endian)

static const uint8_t STS_BROADCAST_ID    = 0xFE;

class STSBus {
 public:
  // `serial` is a hardware UART already wired to the bus driver board.
  explicit STSBus(HardwareSerial& serial) : _s(serial) {}

  // Open the bus. On ESP32 the rx/tx pins select UART2 lines.
  void begin(uint32_t baud = 1000000, int rxPin = 16, int txPin = 17) {
    _s.begin(baud, SERIAL_8N1, rxPin, txPin);
    _s.setTimeout(5);  // ms; reads are short
  }

  // Enable/disable torque on one servo (or STS_BROADCAST_ID for all).
  void torqueEnable(uint8_t id, bool on) {
    writeByte(id, STS_TORQUE_ENABLE, on ? 1 : 0);
  }

  // Write a single 2-byte goal position (and optional speed) to one servo.
  void writePosition(uint8_t id, uint16_t pos, uint16_t speed = 0) {
    uint8_t params[6];
    params[0] = STS_GOAL_POSITION;
    params[1] = pos & 0xFF;          // little-endian: low byte first
    params[2] = (pos >> 8) & 0xFF;
    // Goal_Speed lives at reg 46; goal position is 42..43, so 44..45 are
    // Acceleration-related — to keep it simple and reliable we write speed in a
    // second packet rather than assuming a contiguous block.
    sendPacket(id, STS_INST_WRITE, params, 3);
    if (speed) writeWord(id, STS_GOAL_SPEED, speed);
  }

  // SYNC_WRITE all goal positions in one packet (smoothest — every servo
  // latches its new goal on the same instruction). `ids`/`positions` are
  // parallel arrays of length `count`.
  void syncWritePositions(const uint8_t* ids, const uint16_t* positions, uint8_t count) {
    // SYNC_WRITE params: ADDR  LEN_PER_SERVO  [ID b0 b1] [ID b0 b1] ...
    const uint8_t lenPer = 2;  // 2 data bytes per servo (goal position)
    uint8_t params[3 + 16 * (1 + 2)];  // header + up to 16 servos
    uint8_t n = 0;
    params[n++] = STS_GOAL_POSITION;
    params[n++] = lenPer;
    for (uint8_t i = 0; i < count; i++) {
      params[n++] = ids[i];
      params[n++] = positions[i] & 0xFF;          // low byte first
      params[n++] = (positions[i] >> 8) & 0xFF;
    }
    sendPacket(STS_BROADCAST_ID, STS_INST_SYNC_WRITE, params, n);
  }

  // Read present position (0..4095) from one servo. Returns -1 on timeout/error.
  int readPosition(uint8_t id) {
    uint8_t params[2] = {STS_PRESENT_POSITION, 2};  // addr, num bytes to read
    drainInput();
    sendPacket(id, STS_INST_READ, params, 2);
    uint8_t data[2];
    if (!readResponse(id, data, 2)) return -1;
    return (int)(data[0] | (data[1] << 8));  // little-endian
  }

  // Ping a servo; true if it answers.
  bool ping(uint8_t id) {
    drainInput();
    sendPacket(id, STS_INST_PING, nullptr, 0);
    uint8_t dummy[1];
    return readResponse(id, dummy, 0);
  }

  // ---- low-level register writes ----
  void writeByte(uint8_t id, uint8_t reg, uint8_t value) {
    uint8_t params[2] = {reg, value};
    sendPacket(id, STS_INST_WRITE, params, 2);
  }

  void writeWord(uint8_t id, uint8_t reg, uint16_t value) {
    uint8_t params[3] = {reg, (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF)};
    sendPacket(id, STS_INST_WRITE, params, 3);
  }

 private:
  HardwareSerial& _s;

  void sendPacket(uint8_t id, uint8_t inst, const uint8_t* params, uint8_t numParams) {
    uint8_t len = numParams + 2;  // params + INST + CHECKSUM
    uint8_t checksum = id + len + inst;
    _s.write((uint8_t)0xFF);
    _s.write((uint8_t)0xFF);
    _s.write(id);
    _s.write(len);
    _s.write(inst);
    for (uint8_t i = 0; i < numParams; i++) {
      _s.write(params[i]);
      checksum += params[i];
    }
    _s.write((uint8_t)(~checksum & 0xFF));
    _s.flush();  // ensure the frame is fully out before the bus turns around
  }

  void drainInput() {
    while (_s.available()) _s.read();
  }

  // Read a status packet: 0xFF 0xFF ID LEN ERR [DATA...] CHECKSUM.
  // Copies `expect` data bytes into `out`. Returns false on timeout/mismatch.
  bool readResponse(uint8_t id, uint8_t* out, uint8_t expect) {
    // Find header 0xFF 0xFF.
    uint32_t start = millis();
    int state = 0;
    while (millis() - start < 20) {
      if (!_s.available()) continue;
      uint8_t b = _s.read();
      if (state == 0) state = (b == 0xFF) ? 1 : 0;
      else if (state == 1) { if (b == 0xFF) { state = 2; break; } else state = 0; }
    }
    if (state != 2) return false;

    if (!waitBytes(3 + expect, start)) return false;  // ID LEN ERR + data
    uint8_t rid = _s.read();
    uint8_t len = _s.read();
    uint8_t err = _s.read();
    (void)err;
    if (rid != id) return false;
    uint8_t checksum = rid + len + err;
    for (uint8_t i = 0; i < expect; i++) {
      uint8_t d = _s.read();
      out[i] = d;
      checksum += d;
    }
    if (!waitBytes(1, start)) return false;
    uint8_t recvSum = _s.read();
    return recvSum == (uint8_t)(~checksum & 0xFF);
  }

  bool waitBytes(uint8_t n, uint32_t start) {
    while (_s.available() < n) {
      if (millis() - start > 20) return false;
    }
    return true;
  }
};
