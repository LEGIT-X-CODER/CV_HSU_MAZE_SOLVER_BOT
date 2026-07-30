# 🤖 Rescue Maze Bot - Complete Build Guide

A two-processor autonomous maze-solving robot for **RoboCup Rescue Maze** competitions.
Arduino Uno handles all real-time motion and sensors. Raspberry Pi 4 handles vision and navigation decisions.

---

## 📁 Project File Structure

```
DPS/
├── RescueMaze_Arduino/
│   └── RescueMaze_Arduino.ino      ← Upload to Arduino Uno
└── RescueMaze_Pi/
    ├── rescue_maze_pi.py           ← Run on Raspberry Pi 4
    └── hsu_detector_test.py        ← Test H/S/U on your laptop
```

---

## ⚡ Power Wiring (CRITICAL - Do This First!)

### Step 1: Set XL4015 Output Voltage
1. Connect 3S LiPo to XL4015 IN+ and IN−
2. Using a multimeter on XL4015 OUT+ and OUT−
3. Turn the **adjustment potentiometer** until output reads **5.1V exactly**
4. Do NOT connect anything to output until this is set

### Step 2: Power Rails

```
3S LiPo Battery (11.1V - 12.6V)
    │
    ├──────────────────────────► L298N  VIN  (Raw battery voltage for motors)
    │
    └──► XL4015 Buck Converter IN+
              │
              └──► XL4015 OUT+ (5.1V) ──┬──► Raspberry Pi 4 GPIO Pin 2 or 4 (5V)
                                         ├──► Servo Motor VCC (Red Wire)
                                         ├──► WS2811 LED Strip VCC
                                         ├──► TCS3200 VCC
                                         └──► VL53L0X VCC (via Arduino 5V pin)

⚠️  COMMON GROUND: Wire ALL GND pins together:
    Battery (−), XL4015 OUT−, L298N GND, Arduino GND, Raspberry Pi 4 GND (Pin 6)
```

---

## 🔌 Arduino Uno Wiring

### 3x VL53L0X ToF Sensors (I2C + XSHUT)

| VL53L0X Pin | Arduino Pin | Notes |
|-------------|-------------|-------|
| VCC (all 3) | 5V          | Share power rail |
| GND (all 3) | GND         | Share ground |
| SDA (all 3) | A4          | Shared I2C data |
| SCL (all 3) | A5          | Shared I2C clock |
| XSHUT Left  | **D2**      | Address assignment |
| XSHUT Front | **D3**      | Address assignment |
| XSHUT Right | **D4**      | Address assignment |

> ⚠️ All 3 sensors start with XSHUT=LOW (reset). The code pulls them HIGH one by one
> and assigns unique I2C addresses 0x30, 0x31, 0x32 before the main loop.

### MPU6500 IMU (I2C - Shared with VL53L0X)

| MPU6500 Pin | Arduino Pin |
|-------------|-------------|
| VCC         | 3.3V        |
| GND         | GND         |
| SDA         | A4          |
| SCL         | A5          |
| AD0         | GND         | (Sets I2C address to 0x68) |

### TCS3200 Color Sensor

| TCS3200 Pin | Arduino Pin |
|-------------|-------------|
| VCC         | 5V          |
| GND         | GND         |
| OUT         | D11         |
| S2          | D12         |
| S3          | D13         |
| S0          | A0          |
| S1          | A1          |

> Place TCS3200 facing **downward** pointing at the floor, 5-10mm above ground.

### L298N Motor Driver

| L298N Pin   | Arduino Pin | Notes |
|-------------|-------------|-------|
| ENA         | D5 (PWM)    | Left motor speed |
| IN1         | D6          | Left motor direction A |
| IN2         | D7          | Left motor direction B |
| IN3         | D8          | Right motor direction A |
| IN4         | D9          | Right motor direction B |
| ENB         | D10 (PWM)   | Right motor speed |
| VIN         | Battery (+) | Raw 3S voltage for motors |
| GND         | Common GND  | |
| Motor A Out | Left Motor  | |
| Motor B Out | Right Motor | |

### WS2811 RGB LED Strip

| LED Pin | Arduino Pin |
|---------|-------------|
| VCC     | 5V rail (NOT Arduino 5V - use XL4015 directly) |
| GND     | Common GND  |
| DATA    | A2          |

### Servo Motor (Med Kit Dispenser)

| Servo Pin | Arduino Pin |
|-----------|-------------|
| VCC (Red) | 5V rail (XL4015) |
| GND (Blk) | Common GND  |
| Signal    | A3          |

---

## 🍓 Raspberry Pi 4 Wiring

### OLED Display SSD1306 (I2C)

| OLED Pin | Pi 4 Pin | GPIO |
|----------|----------|------|
| VCC      | Pin 1    | 3.3V |
| GND      | Pin 6    | GND  |
| SDA      | Pin 3    | GPIO2 |
| SCL      | Pin 5    | GPIO3 |

### CSI Camera Module (Rev 1.3)

Connect ribbon cable to the **CAMERA** CSI port on the Pi 4.
(Gently lift the latch, insert ribbon cable (contacts facing away from HDMI ports), press latch down)

### Flashlight LED

Connect flashlight through a **2N2222 NPN transistor** or **5V relay module**:
```
Pi GPIO18 (Pin 12) ──► 330Ω Resistor ──► Transistor Base
Transistor Emitter ──► GND
Transistor Collector ──► Flashlight GND
Flashlight VCC ──► 5V rail
```

### Touch Sensor

| Touch Sensor Pin | Pi 4 Pin | GPIO |
|------------------|----------|------|
| VCC              | Pin 1    | 3.3V |
| GND              | Pin 6    | GND  |
| SIG / OUT        | Pin 11   | GPIO17 |

### Arduino to Pi 4 Connection

**Just use a standard USB-A to USB-B cable!**
- Plug USB-B end into Arduino Uno
- Plug USB-A end into any Raspberry Pi 4 USB port
- Pi 4 will see it as `/dev/ttyACM0`

---

## 💻 Software Setup

### On Arduino Uno

1. Open Arduino IDE
2. Go to **Tools → Manage Libraries**
3. Install these two libraries:
   - Search `VL53L0X` → Install **"VL53L0X" by Pololu**
   - Search `FastLED` → Install **"FastLED" by Daniel Garcia**
4. Open `RescueMaze_Arduino.ino`
5. Select **Board: Arduino Uno** and correct **COM Port**
6. Click **Upload**
7. Open Serial Monitor at **115200 baud** - you should see `READY`

### On Raspberry Pi 4

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Python libraries
pip3 install pyserial opencv-python RPi.GPIO luma.oled

# Enable I2C and Camera
sudo raspi-config
# → Interface Options → I2C → Enable
# → Interface Options → Camera → Enable
# → Reboot

# Enable camera in v4l2 (for OpenCV to access CSI cam)
sudo modprobe bcm2835-v4l2

# Run the bot
python3 rescue_maze_pi.py
```

### On Your Laptop (HSU Detector Testing)

```bash
# Install OpenCV only
pip install opencv-python

# Run with webcam
python hsu_detector_test.py

# Run with a saved image
python hsu_detector_test.py --image my_photo.jpg

# Adjust threshold if needed (default=100, lower for dark rooms)
python hsu_detector_test.py --threshold 80
```

#### Laptop Test Controls

| Key     | Action |
|---------|--------|
| `SPACE` | Freeze current frame and run H/S/U detection |
| `R`     | Resume live webcam feed |
| `Q`/`ESC` | Quit the program |

---

## 📡 Arduino ↔ Pi 4 Serial Commands

### Pi 4 → Arduino (Commands)

| Command        | What Arduino Does |
|----------------|-------------------|
| `FORWARD\n`    | Drive forward one tile (~30cm). Reports back on completion. |
| `TURN_LEFT\n`  | Precise 90° left turn using MPU6500 gyro. |
| `TURN_RIGHT\n` | Precise 90° right turn using MPU6500 gyro. |
| `U_TURN\n`     | Precise 180° turn using MPU6500 gyro. |
| `DISPENSE:2\n` | Drop 2 med-kits via servo. |
| `DISPENSE:1\n` | Drop 1 med-kit via servo. |
| `DISPENSE:0\n` | No kit drop (but flash LED for U victim). |
| `PING\n`       | Connection test. |

### Arduino → Pi 4 (Responses)

| Response                | Meaning |
|-------------------------|---------|
| `READY`                 | Arduino booted successfully, all sensors OK |
| `STOPPED_WALL:L,R`      | Hit a front wall. L=left distance (cm), R=right distance (cm) |
| `STOPPED_OPEN:L,R`      | Completed tile drive, no wall hit. L/R = side distances |
| `EVENT:BLACK_TILE`      | Black hazard detected! Arduino auto-reversed & U-turned. |
| `EVENT:BLUE_TILE`       | Blue puddle detected! Arduino waited 5 seconds. |
| `TURN_COMPLETE`         | Turn finished successfully |
| `DISPENSE_DONE`         | Kit(s) dispensed successfully |
| `PONG`                  | Response to PING |
| `ERR:LeftToF` etc.      | Sensor initialization error on startup |

---

## 🔧 Calibration Steps

### Step 1: Gyro Zero-Rate Offset (MPU6500)
1. Place the robot on a flat surface, perfectly still
2. Open Arduino Serial Monitor (115200)
3. Send `PING` to verify connection
4. Temporarily add this code to `loop()`:
   ```cpp
   Wire.beginTransmission(0x68); Wire.write(0x47); Wire.endTransmission(false);
   Wire.requestFrom(0x68, 2, true);
   int16_t z = (Wire.read()<<8)|Wire.read();
   Serial.println(z); delay(100);
   ```
5. Average ~50 readings while the robot is still
6. That average value = your `GYRO_OFFSET` in the code

### Step 2: TCS3200 Color Thresholds
1. Place the sensor directly over each tile type
2. Watch Serial Monitor output from `readColorPulse()` calls
3. Adjust `BLACK_THRESHOLD` and the blue detection values based on your actual floor tiles

### Step 3: Tile Drive Distance
1. Mark a 30cm tile on the floor
2. Adjust `FORWARD_TILE_MS` in Arduino code until the robot travels exactly one tile

### Step 4: HSU Detector Threshold
```bash
# Use the laptop tool to test your specific lighting conditions:
python hsu_detector_test.py --threshold 80   # Darker room
python hsu_detector_test.py --threshold 120  # Brighter room
```

---

## 🚦 Robot Behaviour Summary

```
Power ON ──► Arduino Boots (Green LED = Ready)
         ──► Pi boots, OLED shows "Touch to START"
         ──► Press Touch Sensor
         ──► Robot starts navigating tile by tile

Each Tile:
  Pi sends FORWARD
  Arduino drives, checking:
    • Black tile → Emergency stop, reverse, U-turn, report BLACK_TILE
    • Blue tile  → Stop 5 seconds, blue LEDs, report BLUE_TILE
    • Front wall → Stop, report STOPPED_WALL with side distances
    • Open tile  → Drive full tile, report STOPPED_OPEN

  At Front Wall:
    Pi → Flashlight ON → Camera frame → H/S/U Detection
    H found → DISPENSE:2
    S found → DISPENSE:1
    U found → DISPENSE:0

  Then Pi decides:
    Left open  → TURN_LEFT
    Right open → TURN_RIGHT
    Both closed → U_TURN

  Loop continues...
```

---

## ❓ Troubleshooting

| Problem | Solution |
|---------|----------|
| `ERR:LeftToF` / `ERR:FrontToF` on startup | Check XSHUT wiring. Make sure each XSHUT pin is connected to the correct Arduino pin and is LOW before init. |
| Robot drifts left/right during driving | Increase `steer` multiplier in `moveForwardTile()`. Check motor wheel symmetry. |
| 90° turn is too much/too little | Calibrate `GYRO_OFFSET` value. Check for `±2.0f` tolerance in `turnMPU()`. |
| Black tile not detected | Measure actual TCS3200 pulse values and update `BLACK_THRESHOLD`. |
| Camera not found on Pi | Run `ls /dev/video*`. If missing, run `sudo modprobe bcm2835-v4l2`. |
| `/dev/ttyACM0` not found | Try `/dev/ttyACM1` or run `ls /dev/tty*` to find correct port. |
| H/S/U always returns NONE | Reduce `--threshold` value. Improve lighting with flashlight. Check `MIN_HEIGHT` value matches your letter size. |
