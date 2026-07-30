#!/usr/bin/env python3
"""
============================================================
  RCJR VISION MASTER - PI STANDALONE TEST SCRIPT
  Raspberry Pi 4 | RPi Camera Rev 1.3 | 180° Flip Enabled

  FEATURES:
  - 180° Camera Rotation (flips upside-down camera feed)
  - RCJRVision Contour Shape-Matching Engine
  - Real-time Terminal Output (Prints H, S, U detections live)

  USAGE (Run via SSH on Pi):
    python3 rcjr_pi_test.py
============================================================
"""

import sys
import time
import cv2
import numpy as np

# RCJR Reference Contours
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
        t0 = time.time()
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

        proc_ms = (time.time() - t0) * 1000.0
        return best_letter, best_conf, best_bbox, scores, proc_ms


def main():
    print("==================================================")
    print("  RCJR VISION TEST ON RASPBERRY PI")
    print("  180° Rotated Camera | Terminal Live Output")
    print("  Press Ctrl+C to Stop")
    print("==================================================\n")

    # Camera Initialization
    cap = None
    picam2 = None
    use_picam2 = False

    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        picam2.start()
        use_picam2 = True
        print("[OK] RPi CSI Camera Rev 1.3 (Picamera2) Ready!")
    except Exception as e:
        print(f"[INFO] Picamera2 init note ({e}). Trying OpenCV fallback...")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    vision = RCJRVisionEngine()
    last_detected = "NONE"
    frame_count = 0

    try:
        while True:
            if use_picam2 and picam2:
                frame = picam2.capture_array()
            elif cap and cap.isOpened():
                ok, frame = cap.read()
                if not ok:
                    continue
            else:
                break

            # ── 180° ROTATION (Fix upside down camera) ────
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            frame_count += 1
            letter, conf, bbox, scores, ms = vision.detect(frame)

            # Print detection live in terminal
            if letter != "NONE":
                print(f"\r[DETECTED VICTIM]  [{letter}]  ({conf:.0f}%)  | Scores: H={scores['H']:.0f}% S={scores['S']:.0f}% U={scores['U']:.0f}%  ({ms:.1f}ms)    ", end="", flush=True)
                last_detected = letter
            else:
                if frame_count % 10 == 0:
                    print(f"\r[SCANNING...] No victim in view... ({ms:.1f}ms)                                        ", end="", flush=True)

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n\n[STOPPED] Exiting test.")
    finally:
        if use_picam2 and picam2:
            picam2.stop()
        if cap:
            cap.release()


if __name__ == "__main__":
    main()
