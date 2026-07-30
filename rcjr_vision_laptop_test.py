#!/usr/bin/env python3
"""
============================================================
  RCJR VISION MASTER - LAPTOP LIVE TEST & BENCHMARK
  RoboCup Junior Rescue HSU Contour Shape-Matching Engine

  FEATURES:
  - Uses RCJRVision Shape-Matching algorithm (cv2.matchShapes)
  - Ultra-fast (No Tesseract overhead ~ 5-15 ms per frame!)
  - Adaptive Thresholding + Dynamic Contour Bounds
  - Displays live confidence bars for H, S, U simultaneously
  - Real-time Webcam GUI overlay

  USAGE:
    python rcjr_vision_laptop_test.py
============================================================
"""

import sys
import os
import time
import platform
import cv2
import numpy as np

# Import reference contours from RCJRVision repo
sys.path.append(os.path.join(os.path.dirname(__file__), "RCJRVision-master"))
try:
    from RCJRVision.params import h_cnt, s_cnt, u_cnt
    print("[OK] Loaded RCJRVision reference contours (H, S, U)")
except Exception as e:
    print(f"[ERROR] Could not import RCJRVision params: {e}")
    sys.exit(1)


class RCJRVisionEnhanced:
    def __init__(self, match_thresh=0.45):
        """
        match_thresh: Lower diff = better match.
        cv2.matchShapes output is typically 0.0 (identical) to 0.5+ (different).
        """
        self.ref_contours = {
            'H': h_cnt,
            'S': s_cnt,
            'U': u_cnt
        }
        self.match_thresh = match_thresh
        self.scaled_dim = 100.0

    def normalize_contour(self, cnt, bbox):
        """Scale and translate contour into a normalized 100x100 bounding box."""
        x, y, w, h = bbox
        if w == 0 or h == 0:
            return None
        shifted = cnt.astype(np.float32) - np.array([x, y], dtype=np.float32)
        scaled = shifted.copy()
        scaled[:, :, 0] = (shifted[:, :, 0] * (self.scaled_dim / float(w)))
        scaled[:, :, 1] = (shifted[:, :, 1] * (self.scaled_dim / float(h)))
        return scaled.astype(np.int32)

    def process_frame(self, frame):
        """
        Scans frame for candidate letter contours and matches shape against H, S, U.
        Returns:
            scores: dict {'H': conf%, 'S': conf%, 'U': conf%}
            best_letter: 'H'|'S'|'U'|'NONE'
            best_conf: float percentage (0.0 to 100.0)
            best_bbox: (x, y, w, h) tuple
            proc_time_ms: float latency in milliseconds
        """
        t0 = time.time()
        fh, fw = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 7, 50, 50)

        # Adaptive thresholding for robust black-on-white text extraction
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
            if area < 300 or area > (fh * fw * 0.7):
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if h < 25 or w < 12:
                continue

            aspect = w / float(h)
            if aspect < 0.25 or aspect > 1.8:
                continue

            # Normalize contour to 100x100
            norm_cnt = self.normalize_contour(cnt, (x, y, w, h))
            if norm_cnt is None:
                continue

            # Compare against each reference letter using Shape Matching (Hu Moments I3)
            for letter, ref_cnt in self.ref_contours.items():
                # cv2.CONTOURS_MATCH_I3 = 3
                diff = cv2.matchShapes(norm_cnt, ref_cnt, cv2.CONTOURS_MATCH_I3, 0)

                # Convert difference score to confidence % (0 diff -> 100%, match_thresh diff -> 0%)
                conf = max(0.0, (1.0 - (diff / self.match_thresh))) * 100.0

                if conf > scores[letter]:
                    scores[letter] = conf

                if diff < min_overall_diff and conf >= 35.0:
                    min_overall_diff = diff
                    best_letter = letter
                    best_conf = conf
                    best_bbox = (x, y, w, h)

        proc_ms = (time.time() - t0) * 1000.0
        return scores, best_letter, best_conf, best_bbox, proc_ms


def draw_gui(frame, scores, best_letter, best_conf, best_bbox, proc_ms):
    disp = frame.copy()
    fh, fw = disp.shape[:2]

    colors = {
        "H": (0, 255, 80),
        "S": (0, 160, 255),
        "U": (255, 80, 220),
        "NONE": (160, 160, 160)
    }

    # ─── Panel ──────────────────────────────────────────────
    px, py, pw, ph = 10, 10, 310, 155
    cv2.rectangle(disp, (px, py), (px + pw, py + ph), (20, 20, 20), -1)
    cv2.rectangle(disp, (px, py), (px + pw, py + ph), (80, 80, 80), 2)
    cv2.putText(disp, "RCJR VISION SHAPE-MATCHING", (px + 10, py + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)

    ry = py + 40
    for ltr in ["H", "S", "U"]:
        sc = scores[ltr]
        bar_w = int((sc / 100.0) * (pw - 90))
        c = colors[ltr] if sc > 0 else (60, 60, 60)
        is_winner = (ltr == best_letter and best_conf >= 35.0)

        # Track background
        cv2.rectangle(disp, (px + 50, ry), (px + pw - 15, ry + 22), (40, 40, 40), -1)
        if bar_w > 0:
            cv2.rectangle(disp, (px + 50, ry), (px + 50 + bar_w, ry + 22), c, -1)

        # Text
        cv2.putText(disp, ltr, (px + 15, ry + 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, colors[ltr] if is_winner else (140, 140, 140), 2)
        cv2.putText(disp, f"{sc:.0f}%", (px + pw - 65, ry + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255) if is_winner else (160, 160, 160), 1)

        if is_winner:
            cv2.putText(disp, "<<", (px + pw - 20, ry + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
        ry += 36

    # ─── Bounding Box ────────────────────────────────────────
    if best_bbox and best_letter != 'NONE' and best_conf >= 35.0:
        bx, by, bw, bh = best_bbox
        c = colors[best_letter]
        cv2.rectangle(disp, (bx, by), (bx + bw, by + bh), c, 3)
        lbl = f"RCJR: {best_letter} ({best_conf:.0f}%)"
        cv2.rectangle(disp, (bx, max(0, by - 30)), (bx + len(lbl) * 15, by), c, -1)
        cv2.putText(disp, lbl, (bx + 4, max(20, by - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)

    # ─── Bottom Bar ─────────────────────────────────────────
    cv2.rectangle(disp, (0, fh - 40), (fw, fh), (15, 15, 15), -1)
    if best_letter != 'NONE' and best_conf >= 35.0:
        cv2.putText(disp, f"DETECTED: {best_letter}  ({best_conf:.0f}%)",
                    (12, fh - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colors[best_letter], 2)
    else:
        cv2.putText(disp, "SCANNING LIVE (RCJR CONTOUR ENGINE)...",
                    (12, fh - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)

    cv2.putText(disp, f"Speed: {proc_ms:.1f} ms | Q = Quit",
                (fw - 240, fh - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    return disp


def benchmark_standard_images():
    """Run accuracy benchmark on generated victim markers."""
    print("\n--- BENCHMARKING RCJR VISION ON STANDARD MARKERS ---")
    detector = RCJRVisionEnhanced()
    markers = ['victim_H_standard', 'victim_S_standard', 'victim_U_standard']
    base_dir = r"C:\Users\AMAN\Desktop\DPS\victim_markers"

    for name in markers:
        path = os.path.join(base_dir, f"{name}.png")
        if not os.path.exists(path):
            continue
        frame = cv2.imread(path)
        scores, letter, conf, bbox, ms = detector.process_frame(frame)
        print(f"Marker {name:22s} -> Detected: [{letter:4s}] ({conf:.0f}%) in {ms:.2f} ms | Scores: H={scores['H']:.0f}% S={scores['S']:.0f}% U={scores['U']:.0f}%")
    print("----------------------------------------------------\n")


def main():
    benchmark_standard_images()

    print("Opening Live Webcam for RCJR Vision Master testing...")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW if platform.system() == "Windows" else 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Could not open webcam.")
        return

    detector = RCJRVisionEnhanced()
    cv2.namedWindow("RCJR Vision Master (Laptop Test)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("RCJR Vision Master (Laptop Test)", 640, 480)

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        scores, letter, conf, bbox, ms = detector.process_frame(frame)
        disp = draw_gui(frame, scores, letter, conf, bbox, ms)

        cv2.imshow("RCJR Vision Master (Laptop Test)", disp)
        if cv2.waitKey(10) & 0xFF in (ord('q'), ord('Q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
