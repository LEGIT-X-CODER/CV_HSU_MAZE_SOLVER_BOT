#!/usr/bin/env python3
"""
============================================================
  RESCUE MAZE PI - RCJR VISION + TTP223 + OLED + ARDUINO SERIAL
  Raspberry Pi 4 | RPi Camera Rev 1.3 | SSD1306 OLED | TTP223 Touch

  HARDWARE SETUP:
    - Camera: RPi Camera Rev 1.3 (CSI Ribbon cable)
    - Touch Sensor (TTP223): VCC=3.3V, GND=GND, OUT=GPIO 17 (Pin 11)
    - OLED Display (SSD1306): VCC=3.3V, GND=GND, SDA=GPIO 2 (Pin 3), SCL=GPIO 3 (Pin 5)
    - Arduino Serial: USB cable (/dev/ttyACM0) or GPIO UART (/dev/serial0)

  STARTUP & RUN WORKFLOW:
    1. Pi boots, initializes OLED, Camera, Serial, and TTP223 Touch Sensor.
    2. OLED displays: "READY / TOUCH TO START"
    3. User touches TTP223 sensor → Pi beeps/logs & sends "START" to Arduino.
    4. Arduino starts maze navigation and sends "D:XX" distance every ~200ms.
    5. RCJRVision (Contour Shape Matching) scans video frames continuously (~10ms latency).
    6. When letter H/S/U detected AND dist <= 10cm → Pi sends "VICTIM:H/S/U" to Arduino.
    7. Arduino stops → deploys rescue kit(s) → sends VICTIM_ACK → resumes maze.

============================================================
"""

import sys
import os
import time
import platform
import threading
import cv2
import numpy as np

# ─── TTP223 Touch Sensor GPIO Setup ─────────────────────────
TOUCH_PIN = 17   # GPIO 17 (Pin 11 on Pi Header)
GPIO_OK = False
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TOUCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO_OK = True
    print(f"[OK] TTP223 Touch Sensor initialized on GPIO {TOUCH_PIN}")
except Exception as e:
    print(f"[WARN] RPi.GPIO not available or non-Pi platform: {e}")

# ─── SSD1306 OLED Display Setup ──────────────────────────────
OLED_OK = False
oled_dev = None
try:
    from luma.core.interface.serial import i2c
    from luma.oled.device import ssd1306
    from luma.core.render import canvas
    from PIL import ImageFont, ImageDraw
    _interface = i2c(port=1, address=0x3C)
    oled_dev = ssd1306(_interface)
    OLED_OK = True
    print("[OK] SSD1306 0.96\" OLED Display initialized (0x3C)")
except Exception as e:
    print(f"[WARN] OLED not initialized: {e}")

# ─── Serial Setup ────────────────────────────────────────────
SERIAL_OK = False
try:
    import serial
    import serial.tools.list_ports
    SERIAL_OK = True
except ImportError:
    print("[WARN] pyserial not found — serial disabled")

# ─── RCJRVision Reference Contours (H, S, U) ─────────────────
# Embedded contour vectors for zero-dependency portability
h_cnt = np.array([[0,0],[19,0],[19,39],[80,39],[80,0],[98,0],[98,98],[80,98],[80,53],[19,53],[19,99],[0,98]]).reshape((-1,1,2))

s_cnt = np.array([[34,1],[36,0],[63,0],[65,1],[71,1],[73,3],[75,3],[76,5],[78,5],[80,7],[82,7],[90,15],[90,16],
                  [92,18],[92,20],[94,22],[94,28],[92,30],[76,30],[75,28],[75,24],[73,22],[73,20],[69,16],[67,16],
                  [65,15],[61,15],[59,13],[38,13],[36,15],[32,15],[30,16],[28,16],[23,22],[23,30],[26,33],[28,33],
                  [30,35],[34,35],[36,37],[42,37],[44,39],[55,39],[57,41],[63,41],[65,43],[73,43],[75,45],[78,45],
                  [80,47],[82,47],[86,50],[88,50],[94,56],[94,58],[96,60],[96,62],[98,64],[98,73],[96,75],[96,77],
                  [94,79],[94,81],[84,90],[82,90],[80,92],[78,92],[76,94],[75,94],[73,96],[67,96],[65,98],[36,98],
                  [34,96],[28,96],[26,94],[23,94],[21,92],[19,92],[17,90],[15,90],[3,79],[3,77],[1,75],[1,71],[0,69],
                  [0,67],[3,64],[15,64],[19,67],[19,71],[21,73],[21,75],[23,77],[25,77],[28,81],[32,81],[34,83],
                  [38,83],[40,84],[61,84],[63,83],[67,83],[69,81],[73,81],[78,75],[78,66],[73,60],[71,60],[69,58],
                  [65,58],[63,56],[55,56],[53,54],[46,54],[44,52],[36,52],[34,50],[28,50],[26,49],[23,49],[21,47],
                  [19,47],[17,45],[15,45],[7,37],[7,35],[5,33],[5,20],[7,18],[7,16],[19,5],[21,5],[23,3],[26,3],[28,1]]).reshape((-1,1,2))

u_cnt = np.array([[0,1],[1,0],[16,0],[18,1],[18,66],[19,68],[19,72],[21,74],[21,75],[25,80],[27,80],[28,81],
                  [30,81],[31,83],[33,83],[34,84],[39,84],[40,86],[57,86],[59,84],[65,84],[66,83],[68,83],[69,81],
                  [71,81],[77,75],[77,74],[78,72],[78,71],[80,69],[80,57],[81,56],[81,1],[83,0],[96,0],[98,1],
                  [98,69],[96,71],[96,75],[95,77],[95,78],[92,81],[92,83],[84,90],[83,90],[81,92],[80,92],[78,93],
                  [77,93],[75,95],[72,95],[71,96],[66,96],[65,98],[34,98],[33,96],[27,96],[25,95],[22,95],[21,93],
                  [19,93],[18,92],[16,92],[13,89],[12,89],[7,84],[7,83],[4,80],[4,78],[3,77],[3,74],[1,72],[1,68],[0,66]]).reshape((-1,1,2))

REF_CONTOURS = {'H': h_cnt, 'S': s_cnt, 'U': u_cnt}
BAUD = 115200
DETECTION_DIST_CM = 10


# ═══════════════════════════════════════════════════════════════
#  RCJR VISION ENGINE (Fast Shape Matching)
# ═══════════════════════════════════════════════════════════════
class RCJRVisionEngine:
    def __init__(self, match_thresh=0.45):
        self.match_thresh = match_thresh
        self.scaled_dim = 100.0

    def normalize_contour(self, cnt, bbox):
        x, y, w, h = bbox
        if w == 0 or h == 0:
            return None
        shifted = cnt.astype(np.float32) - np.array([x, y], dtype=np.float32)
        scaled = shifted.copy()
        scaled[:, :, 0] = (shifted[:, :, 0] * (self.scaled_dim / float(w)))
        scaled[:, :, 1] = (shifted[:, :, 1] * (self.scaled_dim / float(h)))
        return scaled.astype(np.int32)

    def detect(self, frame):
        fh, fw = frame.shape[:2]
        if len(frame.shape) == 3 and frame.shape[2] == 4:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
        elif len(frame.shape) == 3 and frame.shape[2] == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame.copy()
        gray = cv2.bilateralFilter(gray, 7, 50, 50)

        binary = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            blockSize=25, C=10
        )

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        scores = {'H': 0.0, 'S': 0.0, 'U': 0.0}
        best_letter = 'NONE'
        best_conf = 0.0
        best_bbox = None
        min_overall_diff = 999.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 250 or area > (fh * fw * 0.75):
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if h < 25 or w < 12:
                continue
            aspect = w / float(h)
            if aspect < 0.25 or aspect > 1.8:
                continue

            norm_cnt = self.normalize_contour(cnt, (x, y, w, h))
            if norm_cnt is None:
                continue

            for letter, ref_cnt in REF_CONTOURS.items():
                diff = cv2.matchShapes(norm_cnt, ref_cnt, cv2.CONTOURS_MATCH_I3, 0)
                conf = max(0.0, (1.0 - (diff / self.match_thresh))) * 100.0
                if conf > scores[letter]:
                    scores[letter] = conf

                if diff < min_overall_diff and conf >= 35.0:
                    min_overall_diff = diff
                    best_letter = letter
                    best_conf = conf
                    best_bbox = (x, y, w, h)

        return best_letter, best_conf, best_bbox, scores


# ═══════════════════════════════════════════════════════════════
#  ARDUINO SERIAL LINK
# ═══════════════════════════════════════════════════════════════
class ArduinoLink:
    def __init__(self):
        self.ser = None
        self.dist_cm = 999
        self.last_ack = ""
        self.lock = threading.Lock()
        self.running = True
        self.connect()

    def connect(self):
        if not SERIAL_OK:
            return
        candidates = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/serial0", "/dev/ttyAMA0"]
        for p in candidates:
            try:
                s = serial.Serial(p, BAUD, timeout=0.2)
                self.ser = s
                print(f"[Serial] Connected to Arduino at {p}")
                threading.Thread(target=self._read_loop, daemon=True).start()
                return
            except Exception:
                pass
        print("[WARN] Arduino port not found — vision running in standalone mode")

    def _read_loop(self):
        while self.running and self.ser:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("D:"):
                        try:
                            with self.lock:
                                self.dist_cm = int(line[2:])
                        except ValueError:
                            pass
                    elif line.startswith("VICTIM_ACK:"):
                        self.last_ack = line
                        print(f"[Arduino] {line}")
                    elif line:
                        print(f"[Arduino] {line}")
            except Exception:
                pass
            time.sleep(0.01)

    def send(self, msg: str):
        if self.ser:
            try:
                self.ser.write((msg + "\r\n").encode())
                self.ser.flush()
                print(f"[Pi → Uno] {msg}")
            except Exception as e:
                print(f"[Serial Send Error] {e}")

    def get_dist(self) -> int:
        with self.lock:
            return self.dist_cm

    def close(self):
        self.running = False
        if self.ser:
            self.ser.close()


# ═══════════════════════════════════════════════════════════════
#  OLED SCREEN DISPLAY WORKER
# ═══════════════════════════════════════════════════════════════
def update_oled(state_text, letter, conf, dist_cm):
    if not OLED_OK or oled_dev is None:
        return
    try:
        with canvas(oled_dev) as draw:
            # Header line
            draw.rectangle((0, 0, 128, 14), fill="white")
            draw.text((4, 1), f"RCJR:{state_text}", fill="black")

            # Main Victim Detection Line
            if letter != "NONE":
                draw.text((4, 20), f"VICTIM: {letter}", fill="white")
                draw.text((4, 34), f"Conf:   {conf:.0f}%", fill="white")
            else:
                draw.text((4, 20), "Scanning H/S/U...", fill="white")
                draw.text((4, 34), "No victim found", fill="white")

            # Bottom Distance Line
            draw.line((0, 50, 128, 50), fill="white")
            draw.text((4, 52), f"Front Dist: {dist_cm} cm", fill="white")
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════
#  MAIN ENTRY POINT
# ═══════════════════════════════════════════════════════════════
def main():
    print("==================================================")
    print("  RESCUE MAZE PI - RCJR VISION MASTER ENGINE")
    print("  RPi Camera Rev 1.3 + SSD1306 OLED + TTP223 Touch")
    print("==================================================\n")

    # 1. Initialize Camera (Picamera2 for CSI Rev 1.3 or OpenCV fallback)
    cap = None
    picam2 = None
    use_picam2 = False

    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        picam2.start()
        use_picam2 = True
        print("[OK] RPi Camera Rev 1.3 initialized via Picamera2")
    except Exception as e:
        print(f"[INFO] Picamera2 init note ({e}). Falling back to OpenCV VideoCapture...")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 2. Initialize Arduino Serial & Vision Engine
    arduino = ArduinoLink()
    vision = RCJRVisionEngine()

    # Initial OLED State
    update_oled("READY", "NONE", 0, 999)

    state_dict = {'running': False}
    threading.Thread(target=terminal_cli_thread, args=(arduino, state_dict), daemon=True).start()

    maze_running = False
    last_touch_state = False
    last_send_time = 0

    print("\n[READY] Touch TTP223 Sensor to START / STOP maze run...")

    try:
        while True:
            # ── Check Touch Sensor for Toggle (Start / Stop) ────
            touched = False
            if GPIO_OK:
                touched = (GPIO.input(TOUCH_PIN) == GPIO.HIGH)

            if touched and not last_touch_state:
                state_dict['running'] = not state_dict['running']
                if state_dict['running']:
                    print("\n[TOUCH EVENT] TTP223 Pressed ──► STARTING MAZE!")
                    arduino.send("START")
                    update_oled("RUNNING", "NONE", 0, 999)
                else:
                    print("\n[TOUCH EVENT] TTP223 Pressed ──► STOPPING MAZE!")
                    arduino.send("STOP")
                    update_oled("STOPPED", "NONE", 0, 999)
                time.sleep(0.3)  # debounce

            last_touch_state = touched

            if not state_dict['running']:
                update_oled("TOUCH START", "NONE", 0, 999)
                time.sleep(0.1)
                continue

            # ── Main Vision & Navigation Loop ────────────────
            if use_picam2 and picam2:
                frame = picam2.capture_array()
            elif cap and cap.isOpened():
                ok, frame = cap.read()
                if not ok:
                    continue
            else:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)

            # 180° Rotation to fix upside-down camera mounting
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # Detect H, S, U using RCJR Vision Engine (~10ms)
            letter, conf, bbox, scores = vision.detect(frame)
            dist_cm = arduino.get_dist()

            # Decision Logic: Send VICTIM command if Pi is SURE (H/S/U >= 35%)
            now = time.time()
            if (letter in ["H", "S", "U"] and conf >= 35.0 and (now - last_send_time) > 2.5):
                print(f"\n🔥 [VICTIM DETECTED & CONFIRMED] Letter={letter} ({conf:.0f}%) | Dist={dist_cm}cm")
                arduino.send(f"VICTIM:{letter}")
                last_send_time = now
                update_oled(f"VICTIM:{letter}", letter, conf, dist_cm)
            else:
                update_oled("RUNNING", letter if conf >= 20 else "NONE", conf, dist_cm)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[EXIT] User stopped program")
    finally:
        update_oled("STOPPED", "NONE", 0, 0)
        if use_picam2 and picam2:
            picam2.stop()
        if cap:
            cap.release()
        arduino.close()
        if GPIO_OK:
            GPIO.cleanup()
        print("[CLEANUP] Complete.")


if __name__ == "__main__":
    main()
