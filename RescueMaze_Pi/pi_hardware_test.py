#!/usr/bin/env python3
"""
============================================================
  PI HARDWARE TEST SUITE - TOUCH SENSOR, OLED, CAMERA, SERIAL
  Raspberry Pi 4

  TESTS:
    1. TTP223 Touch Sensor (GPIO 17 / Pin 11)
    2. SSD1306 0.96" OLED Display (I2C 0x3C)
    3. RPi Camera Rev 1.3 (Picamera2 / OpenCV)
    4. Arduino Serial Communication (/dev/ttyACM0)

  USAGE:
    python3 pi_hardware_test.py
============================================================
"""

import sys
import time
import cv2
import numpy as np

TOUCH_PIN = 17

print("==================================================")
print("  RASPBERRY PI 4 - HARDWARE DIAGNOSTIC TEST")
print("==================================================\n")

# 1. TEST OLED DISPLAY
print("[1/4] Testing SSD1306 0.96\" OLED Display (I2C 0x3C)...")
try:
    from luma.core.interface.serial import i2c
    from luma.oled.device import ssd1306
    from luma.core.render import canvas

    serial_i2c = i2c(port=1, address=0x3C)
    device = ssd1306(serial_i2c)

    with canvas(device) as draw:
        draw.rectangle((0, 0, 128, 64), outline="white", fill="black")
        draw.text((10, 10), "OLED TEST OK!", fill="white")
        draw.text((10, 30), "Rescue Maze Pi", fill="white")
        draw.text((10, 48), "Status: READY", fill="white")
    print("  --> OLED DISPLAY SUCCESS! (Check screen)")
except Exception as e:
    print(f"  --> OLED TEST FAILED: {e}")

time.sleep(1.0)

# 2. TEST TTP223 TOUCH SENSOR
print("\n[2/4] Testing TTP223 Touch Sensor on GPIO 17...")
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TOUCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    print("  --> Touch the TTP223 sensor now! (Testing for 5 seconds...)")
    end_time = time.time() + 5.0
    touch_detected = False

    while time.time() < end_time:
        if GPIO.input(TOUCH_PIN) == GPIO.HIGH:
            print("  --> TOUCH DETECTED! (GPIO 17 HIGH)")
            touch_detected = True
            time.sleep(0.3)
        time.sleep(0.05)

    if not touch_detected:
        print("  --> No touch detected during 5 sec test.")
    GPIO.cleanup()
except Exception as e:
    print(f"  --> GPIO TOUCH TEST FAILED: {e}")

time.sleep(1.0)

# 3. TEST CAMERA REV 1.3
print("\n[3/4] Testing RPi Camera Rev 1.3...")
try:
    from picamera2 import Picamera2
    picam2 = Picamera2()
    picam2.start()
    time.sleep(1.0)
    frame = picam2.capture_array()
    cv2.imwrite("/home/pi/test_pi_cam.jpg", frame)
    picam2.stop()
    print("  --> PICAMERA2 SUCCESS! Captured frame saved to ~/test_pi_cam.jpg")
except Exception as e1:
    print(f"  --> Picamera2 test failed ({e1}). Trying OpenCV VideoCapture...")
    try:
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        if ret:
            cv2.imwrite("/home/pi/test_pi_cam.jpg", frame)
            print("  --> OPENCV CAMERA SUCCESS! Saved to ~/test_pi_cam.jpg")
        else:
            print("  --> OpenCV camera frame capture failed.")
        cap.release()
    except Exception as e2:
        print(f"  --> CAMERA TEST FAILED: {e2}")

time.sleep(1.0)

# 4. TEST ARDUINO SERIAL PORT
print("\n[4/4] Testing Arduino Serial Connection...")
try:
    import serial
    ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/serial0"]
    connected = False
    for p in ports:
        try:
            s = serial.Serial(p, 115200, timeout=1.0)
            print(f"  --> SERIAL SUCCESS! Opened port {p} @ 115200 baud")
            s.close()
            connected = True
            break
        except Exception:
            pass
    if not connected:
        print("  --> ARDUINO SERIAL NOT CONNECTED (Check USB cable)")
except Exception as e:
    print(f"  --> SERIAL TEST FAILED: {e}")

print("\n==================================================")
print("  DIAGNOSTIC TEST COMPLETE")
print("==================================================")
