#!/usr/bin/env python3
"""
============================================================
  RESCUE MAZE - LAPTOP SIMULATOR & TESTER (RCJR VISION)
  
  Runs on your Windows Laptop!
  Connect Arduino to Laptop via USB cable.
  Laptop Webcam detects H / S / U victim letters.
  Rotates camera 180° and communicates with Arduino via Serial.
============================================================
"""

import time
import threading
import cv2
import numpy as np

# ---- OPTIONAL SERIAL IMPORT --------------------------------
SERIAL_OK = False
serial = None
try:
    import serial
    import serial.tools.list_ports
    SERIAL_OK = True
except ImportError:
    print("[Laptop WARN] 'pyserial' not installed — running in standalone camera mode.")
    print("              (To enable Arduino connection, run: pip install pyserial)")

# ---- CONFIGURATION ------------------------------------------
BAUD_RATE          = 115200
VICTIM_DIST_CM     = 30        # Accept inspection at 30cm
ROTATE_CAMERA_180  = True      # Set to True/False to toggle 180° camera rotation
VALID_LETTERS      = {'H', 'S', 'U'}

# ---- RCJR REFERENCE CONTOURS (H, S, U) ----------------------
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

# ---- GLOBALS -------------------------------------------------
front_distance   = 200     # Updated by serial thread
inspect_requested= False
last_victim_sent = 0
victim_cooldown  = 3.5     # Seconds cooldown
running          = True
victims_count    = 0
current_status   = "INITIALIZING..."

# ---- 1. AUTO-DETECT ARDUINO COM PORT -------------------------
arduino = None
if SERIAL_OK:
    try:
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            desc = (p.description or "").upper()
            if "ARDUINO" in desc or "CH340" in desc or "USB" in desc or "COM" in p.device:
                try:
                    arduino = serial.Serial(p.device, BAUD_RATE, timeout=0.1)
                    time.sleep(2)
                    print(f"[Laptop] Connected to Arduino on {p.device} ✓")
                    break
                except Exception:
                    pass
    except Exception as e:
        print(f"[Laptop] Serial search error: {e}")

if not arduino:
    print("[Laptop WARN] Running in Standalone Camera Mode (No Arduino connected).")


# =============================================================
#  BACKGROUND THREAD: LISTEN TO ARDUINO MESSAGES
# =============================================================
def serial_reader_thread():
    global front_distance, inspect_requested, running

    while running:
        if arduino and arduino.in_waiting > 0:
            try:
                line = arduino.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("D:"):
                    front_distance = int(line[2:])
                elif "INSPECT_30CM" in line:
                    inspect_requested = True
                    print("\n[Arduino] Stopped at 30cm for inspection!")
                elif line:
                    print(f"  [Arduino] {line}")
            except Exception:
                pass
        time.sleep(0.01)


# =============================================================
#  RCJR CONTOUR MATCHING HSU DETECTOR
# =============================================================
def detect_hsu(frame):
    if ROTATE_CAMERA_180:
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

    return best_letter, best_conf, frame


# =============================================================
#  SEND VICTIM COMMAND TO ARDUINO
# =============================================================
def send_victim(letter):
    global last_victim_sent, victims_count, current_status
    now = time.time()

    if now - last_victim_sent < victim_cooldown:
        return

    cmd = f"VICTIM:{letter}\n"
    if arduino:
        arduino.write(cmd.encode('utf-8'))
        arduino.flush()
        print(f"\n[Laptop >>> ARDUINO] DISPENSE SENT: VICTIM:{letter}")
    else:
        print(f"\n[Laptop Test] Would send to Arduino: VICTIM:{letter}")

    victims_count += 1
    last_victim_sent = now
    current_status = f"VICTIM {letter} DETECTED! (Dropping Kit)"


def send_continue():
    if arduino:
        arduino.write(b"CONTINUE\n")
        arduino.flush()
        print("[Laptop >>> ARDUINO] Sent: CONTINUE")


# =============================================================
#  MAIN LOOP
# =============================================================
def main():
    global current_status, running, inspect_requested

    if arduino:
        t = threading.Thread(target=serial_reader_thread, daemon=True)
        t.start()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("[ERROR] Cannot open webcam!")
        return

    print("\n==================================================")
    print("  RESCUE MAZE - LAPTOP RCJR HSU DETECTOR RUNNING")
    print("  (180° Rotated Camera + Victim Kit Deploy Link)")
    print("  Press 'q' in video window to exit")
    print("==================================================\n")

    current_status = "SCANNING FOR VICTIMS (H/S/U)..."

    while running:
        ret, raw_frame = cap.read()
        if not ret:
            continue

        letter, conf, rotated_frame = detect_hsu(raw_frame)

        if letter in VALID_LETTERS:
            send_victim(letter)
            inspect_requested = False
        elif inspect_requested:
            current_status = "30cm Inspection: No Victim Found -> Sending CONTINUE"
            send_continue()
            inspect_requested = False

        # ---- DRAW ON-SCREEN OVERLAY ----
        overlay = rotated_frame.copy()
        h, w = overlay.shape[:2]

        # Top Banner
        cv2.rectangle(overlay, (0, 0), (w, 35), (20, 20, 20), -1)
        cv2.putText(overlay, f"Front Dist: {front_distance}cm  |  Victims Dropped: {victims_count}",
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        # Bottom Status Banner
        color = (0, 255, 0) if letter != 'NONE' else (200, 200, 200)
        cv2.rectangle(overlay, (0, h - 50), (w, h), (20, 20, 20), -1)
        cv2.putText(overlay, f"Status: {current_status}",
                    (10, h - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if letter != 'NONE':
            cv2.putText(overlay, f"DETECTED: {letter} ({conf:.0f}%)",
                        (10, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        cv2.imshow("Rescue Maze - RCJR Laptop HSU Detector", overlay)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    running = False
    if arduino:
        arduino.close()
    print("[Laptop] Finished.")


if __name__ == "__main__":
    main()
