// ============================================================
  VL53L0X ToF SENSORS DIAGNOSTIC & I2C ADDRESS SCANNER
  Arduino Uno

  DIAGNOSTIC PURPOSE:
    1. Scans I2C bus for active devices (0x29, 0x30, 0x31, 0x32, 0x68).
    2. Tests XSHUT pins (Left=D7, Front=D2, Right=D4).
    3. Step-by-step address reassignment (Left->0x30, Front->0x31, Right->0x32).
    4. Continuously prints distance (mm & cm) for all 3 sensors.

  PINS:
    - Left ToF XSHUT  --> Pin 7
    - Front ToF XSHUT --> Pin 2
    - Right ToF XSHUT --> Pin 4
    - I2C SDA         --> A4
    - I2C SCL         --> A5

  BAUD RATE: 115200 (Open Serial Monitor at 115200)
============================================================

#include <Wire.h>
#include <VL53L0X.h>

#define XSHUT_LEFT   7
#define XSHUT_FRONT  2
#define XSHUT_RIGHT  4

VL53L0X tofLeft;
VL53L0X tofFront;
VL53L0X tofRight;

bool leftOK  = false;
bool frontOK = false;
bool rightOK = false;

// ── I2C Bus Scanner ─────────────────────────────────────────
void scanI2CBus() {
  Serial.println(F("\n--- Scanning I2C Bus Devices ---"));
  byte count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("  Found device at 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.print(address, HEX);

      if (address == 0x29) Serial.print(F(" (Default VL53L0X)"));
      else if (address == 0x30) Serial.print(F(" (Left ToF)"));
      else if (address == 0x31) Serial.print(F(" (Front ToF)"));
      else if (address == 0x32) Serial.print(F(" (Right ToF)"));
      else if (address == 0x68) Serial.print(F(" (MPU6500 Gyro)"));
      else if (address == 0x3C) Serial.print(F(" (OLED Display)"));

      Serial.println();
      count++;
    }
  }
  if (count == 0) {
    Serial.println(F("  [ERROR] No I2C devices found! Check SDA (A4) / SCL (A5) wiring & Power!"));
  }
  Serial.println(F("--------------------------------\n"));
}


// ── Step-by-step ToF Re-addressing ──────────────────────────
void initToFStepByStep() {
  Serial.println(F("--- Initializing ToF Sensors Step-by-Step ---"));

  // Step 1: Turn off all ToF sensors
  pinMode(XSHUT_LEFT,  OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT,  LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  // Step 2: Initialize LEFT ToF (Pin 7)
  Serial.print(F("[1/3] Booting LEFT ToF (XSHUT Pin 7)... "));
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(30);

  if (tofLeft.init()) {
    tofLeft.setAddress(0x30);
    tofLeft.setTimeout(200);
    tofLeft.startContinuous(50);
    leftOK = true;
    Serial.println(F("SUCCESS -> Set Address 0x30 [OK]"));
  } else {
    leftOK = false;
    Serial.println(F("FAILED! [Check Pin 7 XSHUT & Left ToF Wiring]"));
  }

  // Step 3: Initialize FRONT ToF (Pin 2)
  Serial.print(F("[2/3] Booting FRONT ToF (XSHUT Pin 2)... "));
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(30);

  if (tofFront.init()) {
    tofFront.setAddress(0x31);
    tofFront.setTimeout(200);
    tofFront.startContinuous(50);
    frontOK = true;
    Serial.println(F("SUCCESS -> Set Address 0x31 [OK]"));
  } else {
    frontOK = false;
    Serial.println(F("FAILED! [Check Pin 2 XSHUT & Front ToF Wiring]"));
  }

  // Step 4: Initialize RIGHT ToF (Pin 4)
  Serial.print(F("[3/3] Booting RIGHT ToF (XSHUT Pin 4)... "));
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(30);

  if (tofRight.init()) {
    tofRight.setAddress(0x32);
    tofRight.setTimeout(200);
    tofRight.startContinuous(50);
    rightOK = true;
    Serial.println(F("SUCCESS -> Set Address 0x32 [OK]"));
  } else {
    rightOK = false;
    Serial.println(F("FAILED! [Check Pin 4 XSHUT & Right ToF Wiring]"));
  }

  Serial.println(F("--------------------------------------------\n"));
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);

  Serial.println(F("=================================================="));
  Serial.println(F("   VL53L0X ToF SENSORS DIAGNOSTIC TESTER"));
  Serial.println(F("=================================================="));

  // Step 1: Scan I2C
  scanI2CBus();

  // Step 2: Step-by-Step ToF Init
  initToFStepByStep();

  // Step 3: Final Bus Scan after address assignment
  scanI2CBus();
}


void loop() {
  int leftDist  = 0;
  int frontDist = 0;
  int rightDist = 0;

  if (leftOK) {
    int raw = tofLeft.readRangeContinuousMillimeters();
    leftDist = (tofLeft.timeoutOccurred() || raw > 2000) ? 999 : raw / 10;
  }

  if (frontOK) {
    int raw = tofFront.readRangeContinuousMillimeters();
    frontDist = (tofFront.timeoutOccurred() || raw > 2000) ? 999 : raw / 10;
  }

  if (rightOK) {
    int raw = tofRight.readRangeContinuousMillimeters();
    rightDist = (tofRight.timeoutOccurred() || raw > 2000) ? 999 : raw / 10;
  }

  // Print live distance table
  Serial.print(F("LEFT: "));
  if (leftOK) { Serial.print(leftDist); Serial.print(F(" cm  ")); }
  else { Serial.print(F("[FAIL]  ")); }

  Serial.print(F("| FRONT: "));
  if (frontOK) { Serial.print(frontDist); Serial.print(F(" cm  ")); }
  else { Serial.print(F("[FAIL]  ")); }

  Serial.print(F("| RIGHT: "));
  if (rightOK) { Serial.print(rightDist); Serial.print(F(" cm")); }
  else { Serial.print(F("[FAIL]")); }

  Serial.println();
  delay(300);
}
