# Rescue Maze Raspberry Pi Setup Guide

## 1. Hardware Connections (Raspberry Pi 4)

### A. TTP223 Touch Sensor (Start Button)
- **VCC** ──► Pi 3.3V (Pin 1)
- **GND** ──► Pi GND (Pin 9)
- **OUT** ──► Pi GPIO 17 (Pin 11)

### B. 0.96 inch SSD1306 OLED Display (I2C)
- **VCC** ──► Pi 3.3V (Pin 1)
- **GND** ──► Pi GND (Pin 6)
- **SDA** ──► Pi GPIO 2 / SDA (Pin 3)
- **SCL** ──► Pi GPIO 3 / SCL (Pin 5)

### C. RPi Camera Rev 1.3
- Connect ribbon cable directly into the Pi 4 CSI Camera Port (blue strip facing Ethernet/USB ports).

### D. Arduino Uno (Serial)
- USB Cable: Connect Arduino USB-B to any USB-A port on Raspberry Pi (`/dev/ttyACM0`).

---

## 2. File Transfer to Raspberry Pi

From your laptop PowerShell terminal:
```powershell
scp "C:\Users\AMAN\Desktop\DPS\RescueMaze_Pi\rescue_maze_pi.py" pi@192.168.137.29:~/
scp "C:\Users\AMAN\Desktop\DPS\RescueMaze_Pi\pi_hardware_test.py" pi@192.168.137.29:~/
```

---

## 3. How to Run

### Step 1: Run Hardware Diagnostic Test (Verify OLED, Sensor, Camera & Serial)
```bash
python3 pi_hardware_test.py
```

### Step 2: Run Full System (RCJRVision + Touch + OLED + Arduino)
```bash
python3 rescue_maze_pi.py
```

---

## 4. System Workflow

1. **Boot**: Pi starts, OLED shows `TOUCH TO START`.
2. **Touch**: Touch the TTP223 sensor → Pi beeps and sends `START` to Arduino.
3. **Maze**: Arduino receives `START`, beeps twice, and begins maze navigation while streaming `D:XX` distance over USB serial.
4. **Vision**: RCJRVision Engine scans camera feed at ~10ms per frame using contour shape matching (`cv2.matchShapes`).
5. **Victim Detected**: If letter H, S, or U detected AND distance <= 10cm, Pi sends `VICTIM:H/S/U` to Arduino.
6. **Action**: Arduino stops, deploys rescue kit(s) based on victim type, and resumes maze navigation.
