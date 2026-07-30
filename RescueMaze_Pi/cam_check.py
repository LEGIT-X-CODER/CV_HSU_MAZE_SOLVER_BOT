#!/usr/bin/env python3
"""
Simple Pi CSI Camera Test Script (Rev 1.3)
Captures 1 frame, flips it 180°, and saves to ~/cam_test_flip.jpg
"""

import sys
import time
import cv2

print("==================================================")
print("  TESTING RPI CAMERA REV 1.3 (PICAMERA2)")
print("==================================================\n")

try:
    from picamera2 import Picamera2
    print("[1] Initializing Picamera2...")
    picam2 = Picamera2()
    picam2.start()
    print("[2] Camera started! Waiting 1 sec for auto-exposure...")
    time.sleep(1.0)
    
    frame = picam2.capture_array()
    picam2.stop()
    print(f"[3] Captured frame! Shape = {frame.shape}")

    # Flip 180 degrees
    frame_flipped = cv2.rotate(frame, cv2.ROTATE_180)
    
    cv2.imwrite("/home/pi/cam_test_flip.jpg", frame_flipped)
    print("\n[SUCCESS] Image saved to ~/cam_test_flip.jpg (180° Flipped)!")

except Exception as e:
    print(f"\n[ERROR] Picamera2 failed: {e}")
    print("Trying OpenCV VideoCapture fallback...")
    try:
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        if ret:
            frame_flipped = cv2.rotate(frame, cv2.ROTATE_180)
            cv2.imwrite("/home/pi/cam_test_flip.jpg", frame_flipped)
            print("[SUCCESS via OpenCV] Image saved to ~/cam_test_flip.jpg!")
        else:
            print("[ERROR] OpenCV cap.read() returned False")
        cap.release()
    except Exception as e2:
        print(f"[ERROR] OpenCV fallback failed: {e2}")
