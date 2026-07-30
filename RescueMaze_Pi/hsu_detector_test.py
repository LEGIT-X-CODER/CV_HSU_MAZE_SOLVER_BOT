#!/usr/bin/env python3
"""
HSU Detector - Pi Headless Mode
Black letter (H/S/U) on White background
Works over SSH — no display needed, prints to terminal.
"""

import cv2
import sys
import platform
import threading
import time
import numpy as np

try:
    import pytesseract
    pytesseract.get_tesseract_version()
    print("[OK] Tesseract ready")
except Exception as e:
    print(f"[ERROR] Tesseract not found: {e}")
    sys.exit(1)

# Picamera2 (Pi CSI camera) or fallback to OpenCV USB webcam
USE_PICAMERA = False
try:
    from picamera2 import Picamera2
    USE_PICAMERA = True
    print("[OK] Picamera2 ready (CSI camera)")
except ImportError:
    print("[INFO] Picamera2 not found — using OpenCV webcam")

TARGETS  = {"H", "S", "U"}
CFG_PSM10 = "--psm 10 --oem 3 -c tessedit_char_whitelist=HSU"
CFG_PSM8  = "--psm 8  --oem 3 -c tessedit_char_whitelist=HSU"


def preprocess(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 9, 75, 75)
    th = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        blockSize=25, C=10
    )
    return th, gray


def find_letter_boxes(th, fh, fw):
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        if h < 25 or h > fh * 0.90 or w < 10 or w > fw * 0.90:
            continue
        aspect = w / float(h)
        if aspect < 0.2 or aspect > 2.0:
            continue
        if cv2.contourArea(c) < 200:
            continue
        boxes.append((x, y, w, h))
    boxes.sort(key=lambda b: b[2] * b[3], reverse=True)
    return boxes[:4]


def make_roi(gray, box, fh, fw):
    x, y, w, h = box
    pad = max(10, int(min(w, h) * 0.20))
    x1 = max(0, x - pad);  y1 = max(0, y - pad)
    x2 = min(fw, x + w + pad);  y2 = min(fh, y + h + pad)
    roi = gray[y1:y2, x1:x2]
    if roi.size == 0:
        return None
    scale = 150.0 / roi.shape[0]
    roi = cv2.resize(roi, (max(10, int(roi.shape[1] * scale)), 150),
                     interpolation=cv2.INTER_CUBIC)
    _, roi_bin = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    roi_bin = cv2.copyMakeBorder(roi_bin, 25, 25, 25, 25,
                                 cv2.BORDER_CONSTANT, value=255)
    return roi_bin


def ocr_roi(roi_img):
    for cfg in [CFG_PSM10, CFG_PSM8]:
        try:
            data = pytesseract.image_to_data(
                roi_img, config=cfg,
                output_type=pytesseract.Output.DICT
            )
            for i, txt in enumerate(data["text"]):
                letter = txt.strip().upper()
                conf   = float(data["conf"][i])
                if letter in TARGETS and conf >= 45.0:
                    return letter, conf / 100.0
        except Exception:
            pass
    return "NONE", 0.0


def detect_hsu(frame):
    fh, fw = frame.shape[:2]
    th, gray = preprocess(frame)
    boxes = find_letter_boxes(th, fh, fw)

    for box in boxes:
        roi = make_roi(gray, box, fh, fw)
        if roi is None:
            continue
        letter, conf = ocr_roi(roi)
        if letter != "NONE":
            return letter, conf

    # Fallback: center crop
    cy1, cy2 = int(fh*0.1), int(fh*0.9)
    cx1, cx2 = int(fw*0.1), int(fw*0.9)
    center = gray[cy1:cy2, cx1:cx2]
    scale = 150.0 / center.shape[0]
    center = cv2.resize(center, (max(10, int(center.shape[1]*scale)), 150),
                        interpolation=cv2.INTER_CUBIC)
    _, roi_bin = cv2.threshold(center, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    roi_bin = cv2.copyMakeBorder(roi_bin, 25, 25, 25, 25,
                                 cv2.BORDER_CONSTANT, value=255)
    return ocr_roi(roi_bin)


def main():
    # Try different camera indices for Pi
    cap = None
    for idx in [0, 1, 2]:
        c = cv2.VideoCapture(idx)
        if c.isOpened():
            cap = c
            print(f"[OK] Camera opened at index {idx}")
            break
        c.release()

    if cap is None:
        print("[ERROR] No camera found. Check: ls /dev/video*")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("\n=== HSU Detector — HEADLESS MODE (SSH safe) ===")
    print("Hold White paper with Black H / S / U in front of camera")
    print("Press Ctrl+C to stop\n")

    last_letter = "NONE"
    frame_n = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue

            frame_n += 1

            # Run detection every 3 frames
            if frame_n % 3 == 0:
                letter, conf = detect_hsu(frame)

                if letter != "NONE":
                    # Print every new detection (or same if changed)
                    if letter != last_letter or (frame_n % 30 == 0):
                        print(f"[DETECTED] {letter}  ({conf:.0%})  @ frame {frame_n}")
                    last_letter = letter
                else:
                    if last_letter != "NONE":
                        print(f"[---] No letter")
                    last_letter = "NONE"

            time.sleep(0.03)   # ~30 FPS cap

    except KeyboardInterrupt:
        print("\n[EXIT] Stopped by user")
    finally:
        cap.release()
        print("[DONE]")


if __name__ == "__main__":
    main()
