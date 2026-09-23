"""Put a physical SO-101 (LeRobot) arm on CommandAGI — via the recommended SDK route.

This is the *host* side of a two-piece example:

  * the ESP32 firmware (../firmware) drives the SO-101's 6x Feetech STS3215 serial-bus
    servos in real time and speaks a tiny line-delimited JSON protocol over USB serial;
  * this bridge uses the `commandagi` Python SDK to `register_robot("so101-arm")`, streams a
    USB webcam as the robot's camera, and translates the control actions CommandAGI routes to
    the robot into SO-101 joint goals which it sends to the ESP32.

Anyone with the session link — a person in the web UI, an AI agent, or another developer's SDK
client — then watches the camera and drives the arm in real time. This is the same BYO-robot
pattern as the `commandagi-example-connect-a-virtual-rover` rover, pointed at real hardware.

Run it:

    pip install -r requirements.txt
    export COMMANDAGI_API_KEY=cagi_...        # create one in your CommandAGI dashboard (operator scope)
    export SO101_SERIAL_PORT=/dev/ttyUSB0     # optional; the ESP32's USB serial port
    python so101_bridge.py

Then open the printed session URL to watch and drive your arm. If no ESP32 / camera is attached
the bridge still runs headless (placeholder frame, goals logged) so you can try the wiring end
to end before powering servos.
"""
import json
import os
import threading
import time
from typing import Optional

from commandagi import CommandAGI

# ----------------------------------------------------------------------------
# SO-101 / LeRobot follower joints at servo IDs 1..6.
# ----------------------------------------------------------------------------
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# Per-joint servo-unit center / span (0..4095 over 360 deg). The bridge keeps each
# joint as a normalized value and maps it onto [center - span, center + span]; the
# ESP32 clamps again to its own hard safe limits, so this is a soft mapping only.
# gripper is special-cased to a 0..1 range (0 = closed, 1 = open).
JOINT_CENTER = {"shoulder_pan": 2048, "shoulder_lift": 2400, "elbow_flex": 2048,
                "wrist_flex": 2048, "wrist_roll": 2048, "gripper": 2400}
JOINT_SPAN = {"shoulder_pan": 1100, "shoulder_lift": 900, "elbow_flex": 1100,
              "wrist_flex": 1100, "wrist_roll": 1800, "gripper": 700}
GRIPPER_CLOSED, GRIPPER_OPEN = 1700, 2700

# Rest / home pose in normalized space (gripper slightly open).
HOME = {j: 0.0 for j in JOINTS}
HOME["gripper"] = 0.5


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class SO101Arm:
    """Bridges CommandAGI control actions to the ESP32 over USB serial, and streams a webcam.

    Joint targets are held in normalized space ([-1, 1], gripper 0..1), clamped, converted to
    servo units, and pushed to the ESP32 as a `goal` packet whenever they change.
    """

    def __init__(self) -> None:
        self.targets = dict(HOME)
        self._lock = threading.Lock()
        self._serial = self._open_serial()
        self._camera = self._open_camera()
        self.last_state: Optional[list] = None  # latest present positions from the ESP32
        if self._serial is not None:
            threading.Thread(target=self._read_serial, daemon=True).start()
        self._send_goal()  # push the home pose

    # ---- hardware setup (both optional so the example runs headless) ----
    def _open_serial(self):
        port = os.environ.get("SO101_SERIAL_PORT", "/dev/ttyUSB0")
        try:
            import serial  # pyserial
            s = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(2.0)  # let the ESP32 reset after the port opens
            print(f"[so101] ESP32 connected on {port}")
            return s
        except Exception as e:  # noqa: BLE001 — degrade gracefully
            print(f"[so101] no ESP32 on {port} ({e}); running in dry-run mode (goals logged)")
            return None

    def _open_camera(self):
        try:
            import cv2  # opencv-python
            cap = cv2.VideoCapture(int(os.environ.get("SO101_CAMERA_INDEX", "0")))
            if cap.isOpened():
                print("[so101] webcam opened")
                return cap
            cap.release()
        except Exception as e:  # noqa: BLE001
            print(f"[so101] no webcam ({e}); using placeholder frames")
        return None

    # ---- serial I/O with the ESP32 ----
    def _read_serial(self) -> None:
        buf = b""
        while True:
            try:
                buf += self._serial.read(256)
            except Exception:  # noqa: BLE001
                time.sleep(0.5)
                continue
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    msg = json.loads(line.decode("utf-8", "ignore"))
                except (ValueError, TypeError):
                    continue
                if msg.get("t") == "state":
                    self.last_state = msg.get("pos")
                elif msg.get("t") == "ready":
                    print("[so101] ESP32 ready; servos:", msg.get("servos"))

    def _send(self, obj: dict) -> None:
        if self._serial is None:
            print("[so101] (dry-run) ->", obj)
            return
        try:
            self._serial.write((json.dumps(obj) + "\n").encode())
        except Exception as e:  # noqa: BLE001
            print(f"[so101] serial write failed: {e}")

    def _to_units(self, joint: str, value: float) -> int:
        if joint == "gripper":
            v = clamp(value, 0.0, 1.0)
            return int(GRIPPER_CLOSED + (GRIPPER_OPEN - GRIPPER_CLOSED) * v)
        v = clamp(value, -1.0, 1.0)
        return int(JOINT_CENTER[joint] + JOINT_SPAN[joint] * v)

    def _send_goal(self) -> None:
        with self._lock:
            pos = [self._to_units(j, self.targets[j]) for j in JOINTS]
        self._send({"t": "goal", "pos": pos})

    # ---- CommandAGI control vocabulary ----
    def on_action(self, action: str, payload: dict) -> None:
        # Mobile drive vocabulary (so the existing web/agent drive buttons do something sensible).
        if action == "move":  # jog shoulder_lift up
            self._jog("shoulder_lift", +0.15 * float(payload.get("speed", 0.6)))
        elif action == "back":  # jog shoulder_lift down
            self._jog("shoulder_lift", -0.15 * float(payload.get("speed", 0.5)))
        elif action == "turn":  # jog shoulder_pan left/right
            sign = 1.0 if payload.get("dir", "left") == "left" else -1.0
            self._jog("shoulder_pan", sign * 0.12 * float(payload.get("rate", 0.6)))
        elif action == "stop":  # hold current pose
            self._send_goal()
        elif action == "reset" or action == "home":  # go to rest pose
            with self._lock:
                self.targets = dict(HOME)
            self._send_goal()
        # Arm-native vocabulary (preferred for agents).
        elif action == "joint":  # set one normalized joint
            name, value = payload.get("name"), payload.get("value")
            if name in JOINTS and value is not None:
                self._set(name, float(value))
        elif action == "pose":  # set many joints at once
            for name in JOINTS:
                if name in payload:
                    self._set(name, float(payload[name]), defer=True)
            self._send_goal()
        elif action == "gripper":  # 0 = closed, 1 = open
            self._set("gripper", float(payload.get("value", 0.5)))

    def _jog(self, joint: str, delta: float) -> None:
        lo, hi = (0.0, 1.0) if joint == "gripper" else (-1.0, 1.0)
        with self._lock:
            self.targets[joint] = clamp(self.targets[joint] + delta, lo, hi)
        self._send_goal()

    def _set(self, joint: str, value: float, defer: bool = False) -> None:
        lo, hi = (0.0, 1.0) if joint == "gripper" else (-1.0, 1.0)
        with self._lock:
            self.targets[joint] = clamp(value, lo, hi)
        if not defer:
            self._send_goal()

    # ---- camera ----
    def camera(self) -> bytes:
        if self._camera is not None:
            ok, frame = self._camera.read()
            if ok:
                import cv2
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    return buf.tobytes()
        return self._placeholder_frame()

    def _placeholder_frame(self) -> bytes:
        # A tiny valid JPEG (solid dark gray) so the example still streams headless.
        # Built once and cached.
        if not hasattr(self, "_ph"):
            self._ph = _solid_jpeg(640, 480, (28, 28, 32))
        return self._ph


def _solid_jpeg(w: int, h: int, rgb) -> bytes:
    """Encode a solid-color JPEG without requiring OpenCV/Pillow (uses Pillow if present)."""
    try:
        import io
        from PIL import Image
        img = Image.new("RGB", (w, h), rgb)
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=70)
        return buf.getvalue()
    except Exception:  # noqa: BLE001 — last-resort 1x1 JPEG
        return bytes.fromhex(
            "ffd8ffe000104a46494600010100000100010000ffdb004300080606070605080707"
            "07090908"
            + "0a0c140d0c0b0b0c1912130f141d1a1f1e1d1a1c1c20242e2720222c231c1c2837292c"
            "30313434341f27393d38323c2e333432ffc0000b080001000101011100ffc4001f0000"
            "010501010101010100000000000000000102030405060708090a0bffc400b510000201"
            "03030204030505040400000178010203041105122131410613516107227114328191a1"
            "08234252c1d1e1f0152434607182333a14443b51552566273f04141f15263462a32a5e6"
            "a55865444555546566565252666252525f0ff00ffda0008010100003f00f7fa28a28a2"
            "8affd9"
        )


def main() -> None:
    cagi = CommandAGI()  # reads COMMANDAGI_API_KEY (+ optional COMMANDAGI_BASE_URL)
    arm = SO101Arm()

    bridge = cagi.register_robot("so101-arm")
    print("Arm online. Watch + drive it at:", bridge.session_url)
    print("Press Ctrl-C to take it offline.")
    bridge.run(camera=arm.camera, on_action=arm.on_action, fps=10)


if __name__ == "__main__":
    main()
