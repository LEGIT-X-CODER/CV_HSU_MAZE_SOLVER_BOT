// ============================================================
//  SENSOR TEST - Tests ALL sensors one by one
//  Open Serial Monitor at 115200 baud to see results
//
//  YOUR PIN MAP:
//    ToF Left XSHUT=D7, Front XSHUT=D2, Right XSHUT=D4
//    ToF I2C = A4(SDA), A5(SCL)
//    MPU6500 I2C = A4(SDA), A5(SCL)
//    TCS3200: S0=A1, S1=A2, S2=D12, S3=D11, OUT=D13
//    Motors: Left=D5(fwd)/D6(rev), Right=D9(fwd)/D10(rev)
//    Buzzer=D3, Blue LED=A0
//
//  Library: VL53L0X by Pololu
// ============================================================

#include <Wire.h>
#include <VL53L0X.h>

// ---- PINS ---------------------------------------------------
#define XSHUT_LEFT   7
#define XSHUT_FRONT  2
#define XSHUT_RIGHT  4

#define LEFT_FWD     5
#define LEFT_REV     6
#define RIGHT_FWD    9
#define RIGHT_REV    10

#define BUZZER       3
#define BLUE_LED     A0

#define TCS_S0       A1
#define TCS_S1       A2
#define TCS_S2       12
#define TCS_S3       11
#define TCS_OUT      13

// ---- OBJECTS ------------------------------------------------
VL53L0X tofLeft, tofFront, tofRight;
bool tofLeftOK = false, tofFrontOK = false, tofRightOK = false;
bool mpuOK = false;

const int MPU_ADDR = 0x68;


void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(LEFT_FWD,  OUTPUT); pinMode(LEFT_REV,  OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT); pinMode(RIGHT_REV, OUTPUT);
  pinMode(BUZZER,    OUTPUT);
  pinMode(BLUE_LED,  OUTPUT);
  pinMode(TCS_S0, OUTPUT); pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT); pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);

  // Stop motors
  analogWrite(LEFT_FWD, 0); analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, 0);

  // TCS3200 frequency scaling 20%
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);

  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("   RESCUE MAZE - SENSOR TEST SUITE"));
  Serial.println(F("========================================"));
  Serial.println(F(""));

  // ---- TEST 1: LED & BUZZER ----
  Serial.println(F("[TEST 1] LED & Buzzer"));
  Serial.println(F("  LED ON..."));
  digitalWrite(BLUE_LED, HIGH);
  delay(500);
  Serial.println(F("  LED OFF..."));
  digitalWrite(BLUE_LED, LOW);
  delay(200);
  Serial.println(F("  Buzzer beep..."));
  tone(BUZZER, 2000, 200);
  delay(300);
  Serial.println(F("  >> LED & Buzzer OK\n"));

  // ---- TEST 2: I2C SCAN ----
  Serial.println(F("[TEST 2] I2C Bus Scan"));
  scanI2C();
  Serial.println(F(""));

  // ---- TEST 3: MPU6500 ----
  Serial.println(F("[TEST 3] MPU6500 IMU"));
  testMPU();
  Serial.println(F(""));

  // ---- TEST 4: VL53L0X ToF Sensors ----
  Serial.println(F("[TEST 4] VL53L0X ToF Sensors"));
  initToFSensors();
  Serial.println(F(""));

  // ---- TEST 5: TCS3200 Color Sensor ----
  Serial.println(F("[TEST 5] TCS3200 Color Sensor"));
  Serial.println(F("  (Place sensor over different tiles to test)"));
  Serial.println(F(""));

  // ---- TEST 6: Motors ----
  Serial.println(F("[TEST 6] Motor Test"));
  Serial.println(F("  Type 'M' in Serial Monitor to run motor test"));
  Serial.println(F(""));

  Serial.println(F("========================================"));
  Serial.println(F("  SETUP DONE - Live readings below"));
  Serial.println(F("  Type 'M' = Motor test"));
  Serial.println(F("  Type 'G' = Gyro calibration test"));
  Serial.println(F("========================================\n"));

  tone(BUZZER, 2000, 100); delay(150);
  tone(BUZZER, 3000, 100); delay(150);
}


void loop() {
  // ---- Live sensor readings every 500ms ----
  Serial.println(F("--- LIVE READINGS ---"));

  // ToF distances
  if (tofLeftOK || tofFrontOK || tofRightOK) {
    Serial.print(F("  ToF:  L="));
    if (tofLeftOK) {
      int d = tofLeft.readRangeContinuousMillimeters();
      if (tofLeft.timeoutOccurred()) Serial.print(F("TIMEOUT"));
      else { Serial.print(d / 10); Serial.print(F("cm")); }
    } else Serial.print(F("FAIL"));

    Serial.print(F("  F="));
    if (tofFrontOK) {
      int d = tofFront.readRangeContinuousMillimeters();
      if (tofFront.timeoutOccurred()) Serial.print(F("TIMEOUT"));
      else { Serial.print(d / 10); Serial.print(F("cm")); }
    } else Serial.print(F("FAIL"));

    Serial.print(F("  R="));
    if (tofRightOK) {
      int d = tofRight.readRangeContinuousMillimeters();
      if (tofRight.timeoutOccurred()) Serial.print(F("TIMEOUT"));
      else { Serial.print(d / 10); Serial.print(F("cm")); }
    } else Serial.print(F("FAIL"));
    Serial.println();
  }

  // MPU6500 gyro + accel
  if (mpuOK) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPUAll(ax, ay, az, gx, gy, gz);
    Serial.print(F("  MPU:  Gyro Z="));
    Serial.print(gz);
    Serial.print(F("  Accel X="));
    Serial.print(ax);
    Serial.print(F("  Y="));
    Serial.print(ay);
    Serial.print(F("  Z="));
    Serial.println(az);
  }

  // TCS3200 color
  int r = readColor(LOW,  LOW);   // Red
  int g = readColor(HIGH, HIGH);  // Green
  int b = readColor(LOW,  HIGH);  // Blue

  Serial.print(F("  TCS:  R="));
  Serial.print(r);
  Serial.print(F("  G="));
  Serial.print(g);
  Serial.print(F("  B="));
  Serial.print(b);

  // Identify tile
  if (r > 350 && g > 350 && b > 350) {
    Serial.print(F("  -> BLACK TILE"));
  } else if (b < r - 40 && b < 150) {
    Serial.print(F("  -> BLUE TILE"));
  } else if (r < 80 && g < 80 && b < 80) {
    Serial.print(F("  -> WHITE/SILVER"));
  } else {
    Serial.print(F("  -> NORMAL"));
  }
  Serial.println();
  Serial.println();

  // Check for serial commands
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'M' || c == 'm') runMotorTest();
    if (c == 'G' || c == 'g') runGyroCalibration();
  }

  delay(500);
}


// =============================================================
//  I2C BUS SCANNER
// =============================================================
void scanI2C() {
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  Found device at 0x"));
      if (addr < 16) Serial.print(F("0"));
      Serial.print(addr, HEX);

      if (addr == 0x29) Serial.print(F(" (VL53L0X default)"));
      if (addr == 0x30) Serial.print(F(" (VL53L0X Left)"));
      if (addr == 0x31) Serial.print(F(" (VL53L0X Front)"));
      if (addr == 0x32) Serial.print(F(" (VL53L0X Right)"));
      if (addr == 0x68) Serial.print(F(" (MPU6500)"));
      if (addr == 0x3C) Serial.print(F(" (OLED)"));
      Serial.println();
      found++;
    }
  }
  if (found == 0) Serial.println(F("  NO I2C devices found! Check wiring."));
  else { Serial.print(F("  Total: ")); Serial.print(found); Serial.println(F(" devices")); }
}


// =============================================================
//  MPU6500 TEST
// =============================================================
void testMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);

  if (Wire.available()) {
    byte whoami = Wire.read();
    Serial.print(F("  WHO_AM_I = 0x"));
    Serial.print(whoami, HEX);

    if (whoami == 0x70 || whoami == 0x68) {
      Serial.println(F(" -> MPU6500 confirmed!"));
      // Wake up
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x6B); Wire.write(0);
      Wire.endTransmission(true);
      delay(100);
      mpuOK = true;

      // Quick reading
      int16_t ax, ay, az, gx, gy, gz;
      readMPUAll(ax, ay, az, gx, gy, gz);
      Serial.print(F("  Accel: X=")); Serial.print(ax);
      Serial.print(F(" Y=")); Serial.print(ay);
      Serial.print(F(" Z=")); Serial.println(az);
      Serial.print(F("  Gyro:  X=")); Serial.print(gx);
      Serial.print(F(" Y=")); Serial.print(gy);
      Serial.print(F(" Z=")); Serial.println(gz);
      Serial.println(F("  >> MPU6500 OK"));
    } else {
      Serial.println(F(" -> UNKNOWN CHIP!"));
    }
  } else {
    Serial.println(F("  MPU6500 NOT responding! Check wiring."));
  }
}

void readMPUAll(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Start at ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}


// =============================================================
//  VL53L0X INIT (same address re-assignment as main code)
// =============================================================
void initToFSensors() {
  pinMode(XSHUT_LEFT,  OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT,  LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  // Left → 0x30
  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  if (tofLeft.init()) {
    tofLeft.setAddress(0x30);
    tofLeft.setTimeout(200);
    tofLeft.startContinuous(50);
    tofLeftOK = true;
    Serial.println(F("  Left  ToF -> 0x30 OK"));
  } else {
    Serial.println(F("  Left  ToF -> FAILED! Check XSHUT=D7 wiring"));
  }

  // Front → 0x31
  digitalWrite(XSHUT_FRONT, HIGH); delay(10);
  if (tofFront.init()) {
    tofFront.setAddress(0x31);
    tofFront.setTimeout(200);
    tofFront.startContinuous(50);
    tofFrontOK = true;
    Serial.println(F("  Front ToF -> 0x31 OK"));
  } else {
    Serial.println(F("  Front ToF -> FAILED! Check XSHUT=D2 wiring"));
  }

  // Right → 0x32
  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  if (tofRight.init()) {
    tofRight.setAddress(0x32);
    tofRight.setTimeout(200);
    tofRight.startContinuous(50);
    tofRightOK = true;
    Serial.println(F("  Right ToF -> 0x32 OK"));
  } else {
    Serial.println(F("  Right ToF -> FAILED! Check XSHUT=D4 wiring"));
  }
}


// =============================================================
//  TCS3200 COLOR READ
// =============================================================
int readColor(bool s2, bool s3) {
  digitalWrite(TCS_S2, s2 ? HIGH : LOW);
  digitalWrite(TCS_S3, s3 ? HIGH : LOW);
  return (int)pulseIn(TCS_OUT, LOW, 25000);
}


// =============================================================
//  MOTOR TEST (Type 'M' in Serial Monitor)
// =============================================================
void runMotorTest() {
  Serial.println(F("\n=== MOTOR TEST ==="));

  Serial.println(F("  Left Forward..."));
  analogWrite(LEFT_FWD, 120); analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, 0);  analogWrite(RIGHT_REV, 0);
  delay(1000);
  analogWrite(LEFT_FWD, 0);
  delay(500);

  Serial.println(F("  Left Reverse..."));
  analogWrite(LEFT_FWD, 0);   analogWrite(LEFT_REV, 120);
  delay(1000);
  analogWrite(LEFT_REV, 0);
  delay(500);

  Serial.println(F("  Right Forward..."));
  analogWrite(RIGHT_FWD, 120); analogWrite(RIGHT_REV, 0);
  analogWrite(LEFT_FWD, 0);    analogWrite(LEFT_REV, 0);
  delay(1000);
  analogWrite(RIGHT_FWD, 0);
  delay(500);

  Serial.println(F("  Right Reverse..."));
  analogWrite(RIGHT_FWD, 0);  analogWrite(RIGHT_REV, 120);
  delay(1000);
  analogWrite(RIGHT_REV, 0);
  delay(500);

  Serial.println(F("  Both Forward..."));
  analogWrite(LEFT_FWD, 120);  analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, 120); analogWrite(RIGHT_REV, 0);
  delay(1000);

  // Stop all
  analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, 0);

  Serial.println(F("  >> Motor test done\n"));
}


// =============================================================
//  GYRO CALIBRATION TEST (Type 'G' in Serial Monitor)
// =============================================================
void runGyroCalibration() {
  Serial.println(F("\n=== GYRO CALIBRATION (5 sec) ==="));
  Serial.println(F("  Keep robot PERFECTLY STILL!"));

  long sum = 0;
  int count = 0;
  unsigned long start = millis();

  while (millis() - start < 5000) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    int16_t gz = ((int16_t)Wire.read() << 8) | Wire.read();

    sum += gz;
    count++;

    // Fast blink
    digitalWrite(BLUE_LED, (millis() / 100) % 2);
    delay(5);
  }

  float offset = (float)sum / (float)count;
  digitalWrite(BLUE_LED, LOW);

  Serial.print(F("  Samples: ")); Serial.println(count);
  Serial.print(F("  Offset:  ")); Serial.println(offset);
  Serial.println(F("  Copy this offset value into the main code!"));
  Serial.println(F("  >> Calibration done\n"));

  tone(BUZZER, 2000, 200);
}
