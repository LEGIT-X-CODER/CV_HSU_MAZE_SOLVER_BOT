#!/usr/bin/env python3
"""
============================================================
  VICTIM DETECTION & KIT DEPLOYMENT ISOLATED TEST SUITE
  Raspberry Pi 4 | Camera (180° Rotated) | TTP223 | Arduino
============================================================
  WORKFLOW:
    1. Touch TTP223 Sensor (GPIO 17) to START / PAUSE detection.
    2. Camera captures live frames & rotates them 180°.
    3. HSU Vision Engine matches contours for H, S, and U letters.
    4. Upon detection (confidence >= 35%):
       - Sends VICTIM:H, VICTIM:S, or VICTIM:U over Serial to Arduino.
       - Arduino triggers Kit Drop / LED Flashing immediately!
       - OLED and Terminal display detection result.
============================================================
"""

import sys
import os
import time
import threading
import cv2
import numpy as np

# ─── 1. TTP223 Touch Sensor Setup (GPIO 17) ─────────────────
TOUCH_PIN = 17
GPIO_OK = False
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TOUCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO_OK = True
    print("[OK] TTP223 Touch Sensor initialized on GPIO 17")
except Exception as e:
    print(f"[WARN] RPi.GPIO init warning: {e}")

# ─── 2. SSD1306 OLED Display Setup (0x3C) ────────────────────
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
    print("[OK] SSD1306 0.96\" OLED Display initialized")
except Exception as e:
    print(f"[WARN] OLED display not available: {e}")

def update_oled(line1, line2="", line3=""):
    if not OLED_OK or oled_dev is None:
        return
    try:
        with canvas(oled_dev) as draw:
            draw.text((0, 0),  line1[:20], fill="white")
            draw.text((0, 20), line2[:20], fill="white")
            draw.text((0, 42), line3[:20], fill="white")
    except Exception:
        pass

# ─── 3. Serial Link to Arduino ──────────────────────────────
BAUD = 115200
SERIAL_OK = False
ser = None

def init_serial():
    global ser, SERIAL_OK
    try:
        import serial
        import serial.tools.list_ports
        candidates = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/serial0", "/dev/ttyAMA0"]
        for p in candidates:
            try:
                s = serial.Serial(p, BAUD, timeout=0.2)
                ser = s
                SERIAL_OK = True
                print(f"[OK] Serial connected to Arduino at {p}")
                return
            except Exception:
                pass
        print("[WARN] Arduino serial port not found (Running standalone mode)")
    except ImportError:
        print("[WARN] pyserial not installed")

def send_arduino(cmd):
    if SERIAL_OK and ser:
        try:
            ser.write((cmd + "\r\n").encode())
            ser.flush()
            print(f"  \033[93m[Pi → Uno]\033[0m Sent: {cmd}")
        except Exception as e:
            print(f"  \033[91m[Serial Error]\033[0m {e}")

# ─── 4. RCJR HSU Vision Engine (Fast Contour Matching) ───────
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

def detect_hsu(frame):
    # ── 180 Degree Camera Rotation ──
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    fh, fw = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame.copy()
    gray = cv2.bilateralFilter(gray, 7, 50, 50)

    binary = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 25, 10
    )

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_letter = 'NONE'
    best_conf = 0.0
    min_diff = 999.0

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

        # Scale contour to 100x100 for shape matching
        shifted = cnt.astype(np.float32) - np.array([x, y], dtype=np.float32)
        scaled = shifted.copy()
        scaled[:, :, 0] = (shifted[:, :, 0] * (100.0 / float(w)))
        scaled[:, :, 1] = (shifted[:, :, 1] * (100.0 / float(h)))
        norm_cnt = scaled.astype(np.int32)

        for letter, ref_cnt in REF_CONTOURS.items():
            diff = cv2.matchShapes(norm_cnt, ref_cnt, cv2.CONTOURS_MATCH_I3, 0)
            conf = max(0.0, (1.0 - (diff / 0.45))) * 100.0
            if diff < min_diff and conf >= 35.0:
                min_diff = diff
                best_letter = letter
                best_conf = conf

    return best_letter, best_conf

# ─── 5. Camera Initialization ────────────────────────────────
def init_camera():
    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (640, 480)})
        picam2.configure(config)
        picam2.start()
        print("[OK] RPi Camera initialized via Picamera2")
        return ('picam2', picam2)
    except Exception as e:
        print(f"[WARN] Picamera2 failed ({e}) — falling back to OpenCV VideoCapture")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if cap.isOpened():
            print("[OK] Camera initialized via OpenCV VideoCapture")
            return ('opencv', cap)
        else:
            print("[ERROR] Could not open camera")
            return (None, None)

def get_frame(cam_type, cam_obj):
    if cam_type == 'picam2':
        return cam_obj.capture_array()
    elif cam_type == 'opencv':
        ret, frame = cam_obj.read()
        return frame if ret else None
    return None

# ─── 6. Main Test Loop ───────────────────────────────────────
def main():
    init_serial()
    cam_type, cam_obj = init_camera()
    if not cam_obj:
        print("[ERROR] Camera failed to start. Exiting.")
        sys.exit(1)

    active = False
    last_touch_time = 0
    last_victim_send = 0

    print("\n" + "="*55)
    print("  VICTIM DETECTION & KIT DEPLOYMENT TESTER")
    print("  (180° Rotated Camera + Touch Button + Arduino Deploy)")
    print("="*55)
    print("\n[READY] Touch TTP223 Sensor to START / PAUSE testing...\n")
    update_oled("VICTIM TESTER", "READY", "TOUCH TO START")

    try:
        while True:
            # ── Check TTP223 Touch Button ──
            if GPIO_OK:
                if GPIO.input(TOUCH_PIN) == GPIO.HIGH:
                    now = time.time()
                    if now - last_touch_time > 0.5:
                        active = not active
                        last_touch_time = now
                        if active:
                            print("\n\033[92m[TOUCH] Detection STARTED!\033[0m")
                            update_oled("VICTIM TESTER", "RUNNING...", "SCANNING HSU")
                        else:
                            print("\n\033[93m[TOUCH] Detection PAUSED!\033[0m")
                            update_oled("VICTIM TESTER", "PAUSED", "TOUCH TO RESUME")
                        time.sleep(0.3)

            # ── Vision Detection Loop ──
            if active:
                frame = get_frame(cam_type, cam_obj)
                if frame is not None:
                    letter, conf = detect_hsu(frame)

                    if letter in ['H', 'S', 'U'] and conf >= 35.0:
                        now = time.time()
                        # Cooldown of 3.5 seconds so same victim doesn't re-trigger continuously
                        if now - last_victim_send > 3.5:
                            last_victim_send = now
                            print(f"\n\033[92m🔥 [VICTIM DETECTED]\033[0m Letter = \033[1m{letter}\033[0m ({conf:.1f}%)")
                            
                            # Send command to Arduino
                            cmd = f"VICTIM:{letter}"
                            send_arduino(cmd)

                            kits_map = {'H': '2 KITS', 'S': '1 KIT', 'U': '0 KITS'}
                            update_oled("VICTIM DETECTED!", f"LETTER: {letter} ({conf:.0f}%)", f"DEPLOY: {kits_map[letter]}")

            # ── Read Serial Response from Arduino ──
            if SERIAL_OK and ser and ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"  \033[96m[Uno → Pi]\033[0m {line}")
                except Exception:
                    pass

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[EXIT] Test stopped by user.")
    finally:
        if cam_type == 'picam2':
            try: cam_obj.stop()
            except: pass
        elif cam_type == 'opencv':
            try: cam_obj.release()
            except: pass
        if GPIO_OK:
            try: GPIO.cleanup()
            except: pass
        if SERIAL_OK and ser:
            try: ser.close()
            except: pass

if __name__ == "__main__":
    main()
