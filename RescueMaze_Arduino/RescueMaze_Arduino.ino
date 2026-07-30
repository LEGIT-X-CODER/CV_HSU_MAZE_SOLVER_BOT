// ============================================================
//  RESCUE MAZE BOT - ARDUINO UNO (Independent Maze Solver)
//
//  KEY DESIGN: Arduino runs maze FULLY INDEPENDENTLY.
//  Pi is a HELPER that sends victim commands when it spots
//  H/S/U letters. If Pi crashes, bot keeps navigating!
//
//  ARCHITECTURE:
//    Arduino → Pi:  Sends "D:XX\n" (front distance cm) every 200ms
//    Pi → Arduino:  Sends "VICTIM:H\n" / "VICTIM:S\n" / "VICTIM:U\n"
//                   ONLY when letter detected AND distance < 10cm
//
//  PINS:
//    ToF: Left XSHUT=D7, Front XSHUT=D2, Right XSHUT=D4
//    Motors: Left=D5(fwd)/D6(rev), Right=D9(fwd)/D10(rev)
//    MPU6500: I2C (A4/A5)
//    TCS3200: S0=A1, S1=A2, S2=D12, S3=D11, OUT=D13
//    Buzzer=D3, Blue LED=A0, Servo=D8
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
#define TILE_LED     A0   // Tile LED on A0
#define VICTIM_LED   A3   // Victim LED on A3
#define SERVO_PIN    8    // Med-kit dispenser servo

#define TCS_S0       A1
#define TCS_S1       A2
#define TCS_S2       12
#define TCS_S3       11
#define TCS_OUT      13

// =============================================================
//  MOTOR SPEED & TURN TIMING CONTROLS (Easy Tuning at top)
// =============================================================
int FORWARD_SPEED  = 245;   // Straight driving motor speed (0-255)
int TURN_SPEED     = 255;   // High-torque turning motor speed for heavy bot (0-255)
int LEFT_TURN_MS   = 450;   // Left turn duration in ms (TUNE THIS!)
int RIGHT_TURN_MS  = 450;   // Right turn duration in ms (TUNE THIS!)

// ---- OTHER TUNING PARAMETERS --------------------------------
#define WALL_DIST        10   // cm - front wall stop
#define OPEN_PATH_DIST   20   // cm - side is "open"
#define TILE_DRIVE_MS  1200   // ms to drive ~30cm
#define BLACK_THRESH    160   // TCS pulse threshold for black tile (> 160 is black)
#define DIST_SEND_INTERVAL 200 // ms between distance sends to Pi

// ---- OBJECTS ------------------------------------------------

VL53L0X tofLeft, tofFront, tofRight;
const int MPU_ADDR = 0x68;

// ---- GLOBALS ------------------------------------------------
int distL = 200, distF = 200, distR = 200;
bool leftOK  = false;
bool frontOK = false;
bool rightOK = false;

float gyroZOffset    = 0.0;
float yawAngle       = 0.0;
float targetHeading  = 0.0;        // Multiples of 90 degrees (0, 90, 180, -90)
unsigned long lastGyroTime   = 0;
unsigned long lastDistSend   = 0;  // Throttle distance sends to Pi
bool victimInterrupt = false;      // Set true when Pi sends VICTIM cmd
char victimType      = ' ';        // 'H', 'S', or 'U'
bool mazeStarted     = false;      // True after Pi sends START


// =============================================================
//  SETUP & AUTOMATED BOOT SELF-DIAGNOSTIC
// =============================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Pins Setup
  pinMode(LEFT_FWD,   OUTPUT); pinMode(LEFT_REV,   OUTPUT);
  pinMode(RIGHT_FWD,  OUTPUT); pinMode(RIGHT_REV,  OUTPUT);
  pinMode(BUZZER,     OUTPUT);
  pinMode(TILE_LED,   OUTPUT);
  pinMode(VICTIM_LED, OUTPUT);
  pinMode(SERVO_PIN,  OUTPUT);
  stopMotors();
  servoWrite(0); // Servo rest/pull position (0 deg)

  // TCS3200 Pins Setup
  pinMode(TCS_S0, OUTPUT); pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT); pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);

  Serial.println(F("\n=============================================="));
  Serial.println(F("   RESCUE MAZE BOT - DIRECT MAZE SOLVER READY"));
  Serial.println(F("=============================================="));

  // Quick Sensor Initialization (No long boot tests)
  initToFSensors();
  initMPU();
  calibrateGyro();
  lastGyroTime = micros();

  beep(100); delay(50); beep(100);
  waitForStart();
}


// =============================================================
//  MAIN LOOP — rf9-style continuous wall-following navigation
//  Strategy: Drive straight. If front blocked, compare L vs R
//            and turn toward more open side. Dead end = double left.
// =============================================================
unsigned long blueTileCooldownUntil = 0;
const unsigned long BLUE_COOLDOWN_MS = 2500;

void loop() {
  // ---- Always check for Pi commands (NON-BLOCKING) ----
  checkPiCommand();

  // ---- Handle victim if Pi detected one ----
  if (victimInterrupt) {
    handleVictim();
    victimInterrupt = false;
    victimType = ' ';
  }

  // ---- Update absolute heading ----
  updateYaw();

  // ---- Read all sensors ----
  readDistances();
  sendDistanceToPi();

  // ---- Floor color checks (BEFORE any driving) ----
  // BLACK = dead end: stop, buzz, reverse, double-left turn
  if (isBlackTile()) {
    Serial.println(F("[TILE] BLACK -> Dead End!"));
    stopMotors();
    beep(300);
    reverseSmall();
    turnDegrees(+90.0); gyroDelay(100); turnDegrees(+90.0);
    return;
  }

  // BLUE = checkpoint pause with cooldown so same tile doesn't retrigger
  if (isBlueTile() && millis() > blueTileCooldownUntil) {
    Serial.println(F("[TILE] BLUE -> Blink 5s"));
    stopMotors();
    blinkLED(5000);
    blueTileCooldownUntil = millis() + BLUE_COOLDOWN_MS;
    return;
  }

  // ---- 30cm Victim Inspection Pause ----
  static bool inspectedAt30cm = false;

  if (distF > 0 && distF <= 30 && distF > WALL_DIST && !inspectedAt30cm) {
    stopMotors();
    Serial.println(F("INSPECT_30CM")); // Tell Pi we are stopped at 30cm for inspection
    
    // Wait at 30cm until Pi sends output (VICTIM:H/S/U or CONTINUE)
    bool piResponded = false;
    unsigned long pauseStart = millis();
    while (!piResponded && millis() - pauseStart < 3000) {
      if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();
        
        if (cmd.indexOf("VICTIM:") != -1 || cmd.indexOf("CONTINUE") != -1 || cmd.indexOf("NO_VICTIM") != -1) {
          piResponded = true;
          if (cmd.indexOf("VICTIM:") != -1) {
            int idx = cmd.indexOf("VICTIM:");
            victimType = cmd.charAt(idx + 7);
            handleVictim();
          }
        }
      }
      updateYaw();
      delay(10);
    }
    inspectedAt30cm = true;
  }

  // Reset inspected flag when front path opens up (> 35cm)
  if (distF > 35) {
    inspectedAt30cm = false;
  }

  // ---- Navigation decision (rf9.ino SLRB logic) ----
  bool leftOpen  = (distL >= OPEN_PATH_DIST);
  bool frontOpen = (distF > WALL_DIST);
  bool rightOpen = (distR >= OPEN_PATH_DIST);

  if (frontOpen) {
    // S - Straight first: drive forward with Gyro straight-line correction!
    float angleError = normalizeAngle(yawAngle - targetHeading);
    
    // Proportional correction: error * KP
    int gyroSteer = constrain(angleError * 6.0, -50, 50); 
    
    // Wall centering fallback (if close to walls on both sides)
    int wallSteer = 0;
    if (distL < 25 && distR < 25) {
      wallSteer = (distL - distR) * 2;
    }
    
    // Blend both steering commands (gyro has higher weight for straight line consistency)
    int steer = gyroSteer;
    if (abs(wallSteer) > 2) {
      steer = gyroSteer + (wallSteer / 2);
    }
    steer = constrain(steer, -50, 50);

    analogWrite(LEFT_FWD,  constrain(FORWARD_SPEED + steer, 0, 255));
    analogWrite(LEFT_REV,  0);
    analogWrite(RIGHT_FWD, constrain(FORWARD_SPEED - steer, 0, 255));
    analogWrite(RIGHT_REV, 0);

  } else {
    // Front blocked — STOP and decide turn
    stopMotors();
    gyroDelay(200);

    // Re-read for accurate side distances while stopped
    readDistances();
    sendDistanceToPi();

    Serial.print(F("[NAV] WALL! L=")); Serial.print(distL);
    Serial.print(F(" F=")); Serial.print(distF);
    Serial.print(F(" R=")); Serial.println(distR);

    // Give Pi a moment to detect victim at this wall
    unsigned long waitStart = millis();
    while (millis() - waitStart < 300) {
      checkPiCommand();
      if (victimInterrupt) {
        handleVictim();
        victimInterrupt = false;
        victimType = ' ';
        break;
      }
      updateYaw();
      delay(10);
    }

    // SLRB: compare L vs R like rf9.ino
    if (leftOpen && rightOpen) {
      // Both sides open — turn toward the side with more space
      if (distL >= distR) {
        Serial.println(F("[NAV] Both open -> LEFT has more space"));
        turnDegrees(+90.0);
      } else {
        Serial.println(F("[NAV] Both open -> RIGHT has more space"));
        turnDegrees(-90.0);
      }
    } else if (leftOpen) {
      Serial.println(F("[NAV] Only LEFT open -> turning LEFT"));
      turnDegrees(+90.0);
    } else if (rightOpen) {
      Serial.println(F("[NAV] Only RIGHT open -> turning RIGHT"));
      turnDegrees(-90.0);
    } else {
      Serial.println(F("[NAV] Dead end -> DOUBLE LEFT TURN"));
      turnDegrees(+90.0); gyroDelay(100); turnDegrees(+90.0);
    }
  }
}


// =============================================================
//  HARDWARE DIAGNOSTIC COMMAND EXECUTION
// =============================================================
void runDiagnosticCommand(String cmd) {
  if (cmd.indexOf("TEST_MOTOR_FWD") != -1) {
    Serial.println(F("[TEST] Motor Forward 1s"));
    analogWrite(LEFT_FWD, FORWARD_SPEED); analogWrite(LEFT_REV, 0);
    analogWrite(RIGHT_FWD, FORWARD_SPEED); analogWrite(RIGHT_REV, 0);
    delay(1000); stopMotors();
  }
  else if (cmd.indexOf("TEST_MOTOR_REV") != -1) {
    Serial.println(F("[TEST] Motor Reverse 1s"));
    analogWrite(LEFT_FWD, 0); analogWrite(LEFT_REV, FORWARD_SPEED);
    analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, FORWARD_SPEED);
    delay(1000); stopMotors();
  }
  else if (cmd.indexOf("TEST_MOTOR_LEFT") != -1) {
    Serial.println(F("[TEST] Spin Left 1s"));
    analogWrite(LEFT_FWD, 0); analogWrite(LEFT_REV, TURN_SPEED);
    analogWrite(RIGHT_FWD, TURN_SPEED); analogWrite(RIGHT_REV, 0);
    delay(1000); stopMotors();
  }
  else if (cmd.indexOf("TEST_MOTOR_RIGHT") != -1) {
    Serial.println(F("[TEST] Spin Right 1s"));
    analogWrite(LEFT_FWD, TURN_SPEED); analogWrite(LEFT_REV, 0);
    analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, TURN_SPEED);
    delay(1000); stopMotors();
  }
  else if (cmd.indexOf("TEST_MOTOR_STOP") != -1) {
    stopMotors();
    Serial.println(F("[TEST] Motors Stopped"));
  }
  else if (cmd.indexOf("TEST_TOF") != -1) {
    readDistances();
    Serial.print(F("[TEST TOF] Left=")); Serial.print(distL);
    Serial.print(F(" cm | Front=")); Serial.print(distF);
    Serial.print(F(" cm | Right=")); Serial.print(distR); Serial.println(F(" cm"));
  }
  else if (cmd.indexOf("TEST_COLOR") != -1) {
    int r = readColorPulse(LOW, LOW);
    int g = readColorPulse(HIGH, HIGH);
    int b = readColorPulse(LOW, HIGH);
    Serial.print(F("[TEST TCS3200] Pulse R=")); Serial.print(r);
    Serial.print(F(" G=")); Serial.print(g);
    Serial.print(F(" B=")); Serial.print(b);
    if (isBlackTile()) Serial.println(F(" -> [BLACK HAZARD]"));
    else if (isBlueTile()) Serial.println(F(" -> [BLUE PUDDLE]"));
    else Serial.println(F(" -> [WHITE FLOOR]"));
  }
  else if (cmd.indexOf("TEST_MPU") != -1) {
    int16_t gz = readRawGyroZ();
    float dps = (gz - gyroZOffset) / 65.5;
    Serial.print(F("[TEST MPU] Raw Z=")); Serial.print(gz);
    Serial.print(F(" | Z-Rate=")); Serial.print(dps, 2);
    Serial.print(F(" dps | Yaw=")); Serial.print(yawAngle, 2);
    Serial.print(F(" deg | Target=")); Serial.print(targetHeading, 2); Serial.println(F(" deg"));
  }
  else if (cmd.indexOf("STREAM_MPU") != -1) {
    Serial.println(F("[STREAM MPU] Streaming Live Angle for 15s..."));
    lastGyroTime = micros();
    unsigned long start = millis();
    while (millis() - start < 15000) {
      updateYaw();
      int16_t gz = readRawGyroZ();
      float dps = (gz - gyroZOffset) / 65.5;
      Serial.print(F("MPU_LIVE: Yaw=")); Serial.print(yawAngle, 2);
      Serial.print(F(" deg | Rate=")); Serial.print(dps, 2); Serial.println(F(" dps"));
      delay(30);
    }
  }
  else if (cmd.indexOf("TURN_LEFT") != -1) {
    Serial.println(F("[TEST] Gyro Turn +90 deg Left"));
    turnDegrees(+90.0);
  }
  else if (cmd.indexOf("TURN_RIGHT") != -1) {
    Serial.println(F("[TEST] Gyro Turn -90 deg Right"));
    turnDegrees(-90.0);
  }
  else if (cmd.indexOf("TURN_UTURN") != -1) {
    Serial.println(F("[TEST] Gyro U-Turn 180 deg"));
    turnDegrees(-180.0);
  }
  else if (cmd.indexOf("TEST_BUZZER_LED") != -1) {
    Serial.println(F("[TEST] Beep & LED Flash"));
    digitalWrite(BLUE_LED, HIGH); beep(200);
    digitalWrite(BLUE_LED, LOW); delay(200);
    digitalWrite(BLUE_LED, HIGH); beep(200);
    digitalWrite(BLUE_LED, LOW);
  }
  else if (cmd.indexOf("TEST_SERVO") != -1) {
    Serial.println(F("[TEST] Servo Dispenser (Push 138 deg -> Pull 0 deg)"));
    dispenseKits(1);
  }
  else if (cmd.indexOf("H") != -1 && cmd.indexOf("TOF") == -1) {
    Serial.println(F("[EXECUTING] VICTIM H -> 2 Kits Drop"));
    beep(150); delay(100); beep(150);
    dispenseKits(2);
    Serial.println(F("VICTIM_ACK:H"));
  }
  else if (cmd.indexOf("S") != -1 && cmd.indexOf("TEST") == -1 && cmd.indexOf("START") == -1 && cmd.indexOf("STOP") == -1) {
    Serial.println(F("[EXECUTING] VICTIM S -> 1 Kit Drop"));
    beep(150);
    dispenseKits(1);
    Serial.println(F("VICTIM_ACK:S"));
  }
  else if (cmd.indexOf("U") != -1 && cmd.indexOf("TEST") == -1 && cmd.indexOf("TURN") == -1 && cmd.indexOf("BUS") == -1) {
    Serial.println(F("[EXECUTING] VICTIM U -> 0 Kits (Beep Only)"));
    beep(300);
    Serial.println(F("VICTIM_ACK:U"));
  }
}


// =============================================================
//  WAIT FOR PI START HANDSHAKE
// =============================================================
void waitForStart() {
  stopMotors();
  mazeStarted = false;
  Serial.println(F("READY"));
  Serial.println(F("Waiting for Pi START..."));

  while (!mazeStarted) {
    if (Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      cmd.toUpperCase();
      if (cmd.indexOf("START") != -1) {
        mazeStarted = true;
        yawAngle = 0.0;
        targetHeading = 0.0;
        lastGyroTime = micros();
        Serial.println(F("START received — maze running!"));
        beep(200); gyroDelay(100); beep(200);
      } else {
        runDiagnosticCommand(cmd);
      }
    }
    digitalWrite(BLUE_LED, (millis() / 500) % 2);
    delay(10);
  }
  digitalWrite(BLUE_LED, LOW);
}


// =============================================================
//  CHECK FOR PI COMMANDS (Non-blocking)
// =============================================================
void checkPiCommand() {
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd.indexOf("STOP") != -1 || cmd.indexOf("RESET") != -1) {
      stopMotors();
      Serial.println(F("STOP_ACK"));
      beep(100); delay(50); beep(100);
      waitForStart();
      return;
    }

    runDiagnosticCommand(cmd);

    int idx = cmd.indexOf("VICTIM:");
    if (idx != -1) {
      char v = cmd.charAt(idx + 7);  // 'H', 'S', or 'U'
      if (v == 'H' || v == 'S' || v == 'U') {
        victimType = v;
        victimInterrupt = true;
        Serial.print(F("PI_CMD_RECV:")); Serial.println(victimType);
        handleVictim();  // Execute victim rescue sequence immediately!
      }
    }
  }
}


// =============================================================
//  HANDLE VICTIM (Stop, drop kits, flash LED, then continue)
// =============================================================
void handleVictim() {
  stopMotors();

  if (victimType == 'H') {
    Serial.println(F("VICTIM H -> 2 kits"));
    // 3 beeps = Harmed
    beep(200); gyroDelay(100); beep(200); gyroDelay(100); beep(200);
    dispenseKits(2);
    flashLED(3, 300);

  } else if (victimType == 'S') {
    Serial.println(F("VICTIM S -> 1 kit"));
    // 2 beeps = Stable
    beep(200); gyroDelay(100); beep(200);
    dispenseKits(1);
    flashLED(2, 300);

  } else if (victimType == 'U') {
    Serial.println(F("VICTIM U -> 0 kits"));
    // 1 beep = Unharmed
    beep(200);
    flashLED(1, 500);
  }

  // Send ACK to Pi so simulation confirms victim handled
  Serial.print(F("VICTIM_ACK:"));
  Serial.println(victimType);
  gyroDelay(500);
}


// =============================================================
//  SEND FRONT DISTANCE TO PI (Throttled every 200ms)
//  Format: "D:XX\n" where XX is distance in cm
// =============================================================
void sendDistanceToPi() {
  if (millis() - lastDistSend >= DIST_SEND_INTERVAL) {
    Serial.print(F("D:")); Serial.println(distF);
    lastDistSend = millis();
  }
}


// =============================================================
//  ToF SENSORS
// =============================================================
void initToFSensors() {
  pinMode(XSHUT_LEFT,  OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW); digitalWrite(XSHUT_FRONT, LOW); digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  digitalWrite(XSHUT_LEFT, HIGH); delay(30);
  if (tofLeft.init()) {
    tofLeft.setAddress(0x30);
    tofLeft.setTimeout(200);
    tofLeft.startContinuous(50);
    leftOK = true;
    Serial.println(F("  Left  ToF OK (0x30)"));
  } else {
    leftOK = false;
    Serial.println(F("  Left  ToF FAIL!"));
  }

  digitalWrite(XSHUT_FRONT, HIGH); delay(30);
  if (tofFront.init()) {
    tofFront.setAddress(0x31);
    tofFront.setTimeout(200);
    tofFront.startContinuous(50);
    frontOK = true;
    Serial.println(F("  Front ToF OK (0x31)"));
  } else {
    frontOK = false;
    Serial.println(F("  Front ToF FAIL!"));
  }

  digitalWrite(XSHUT_RIGHT, HIGH); delay(30);
  if (tofRight.init()) {
    tofRight.setAddress(0x32);
    tofRight.setTimeout(200);
    tofRight.startContinuous(50);
    rightOK = true;
    Serial.println(F("  Right ToF OK (0x32)"));
  } else {
    rightOK = false;
    Serial.println(F("  Right ToF FAIL!"));
  }
}

void readDistances() {
  int raw;

  if (leftOK) {
    raw = tofLeft.readRangeContinuousMillimeters();
    distL = (tofLeft.timeoutOccurred() || raw <= 0 || raw > 2000) ? 200 : raw / 10;
  } else {
    distL = 200;
  }

  if (frontOK) {
    raw = tofFront.readRangeContinuousMillimeters();
    distF = (tofFront.timeoutOccurred() || raw <= 0 || raw > 2000) ? 200 : raw / 10;
  } else {
    distF = 200;
  }

  if (rightOK) {
    raw = tofRight.readRangeContinuousMillimeters();
    distR = (tofRight.timeoutOccurred() || raw <= 0 || raw > 2000) ? 200 : raw / 10;
  } else {
    distR = 200;
  }
}


// =============================================================
//  MPU6050/MPU6500 GYRO INTEGRATION & YAW HELPERS
// =============================================================
void initMPU() {
  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0x00); 
  Wire.endTransmission(true);
  delay(50);

  // Configure gyro to 500dps full scale (0x1B = 0x08 -> 65.5 LSB/dps)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); 
  Wire.write(0x08); 
  Wire.endTransmission(true);
  delay(50);

  Serial.println(F("  MPU Gyro Ready (±500 dps, 65.5 LSB/dps)"));
}

float normalizeAngle(float deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

void updateYaw() {
  unsigned long now = micros();
  if (lastGyroTime == 0) {
    lastGyroTime = now;
    return;
  }
  float dt = (now - lastGyroTime) / 1000000.0;
  lastGyroTime = now;
  
  // Ignore anomalies (dt <= 0 or pause > 1 second)
  if (dt <= 0.0 || dt > 1.0) return; 

  int16_t rawZ = readRawGyroZ();
  // Sensitivity is 65.5 LSB/dps at ±500 dps config
  float gyroRate = (float)(rawZ - gyroZOffset) / 65.5;
  yawAngle += gyroRate * dt;
  yawAngle = normalizeAngle(yawAngle);
}

void gyroDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateYaw();
    checkPiCommand();
    delay(1);
  }
}

void calibrateGyro() {
  long sum = 0;
  int count = 1000;
  Serial.println(F("  Calibrating Gyro (1000 samples)... Keep Still!"));
  for (int i = 0; i < count; i++) {
    sum += readRawGyroZ();
    digitalWrite(BLUE_LED, (i / 50) % 2);
    delay(2);
  }
  gyroZOffset = (float)sum / (float)count;
  digitalWrite(BLUE_LED, LOW);
  yawAngle = 0.0;
  targetHeading = 0.0;
}

int16_t readRawGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return (int16_t)gyroZOffset;

  Wire.requestFrom(MPU_ADDR, 2, true);
  if (Wire.available() >= 2) {
    return ((int16_t)Wire.read() << 8) | Wire.read();
  }
  return (int16_t)gyroZOffset;
}

void activeBrake() {
  // Short reverse pulse to kill motor inertia instantly
  analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, 220);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, 220);
  delay(40);
  stopMotors();
}

// =============================================================
//  GYRO-CONTROLLED PRECISION TURNING
// =============================================================
void turnDegrees(float targetAngle) {
  // Update the global target heading to the next 90-degree multiple
  targetHeading = normalizeAngle(targetHeading + targetAngle);

  Serial.print(F("[TURN] Target Heading = ")); Serial.println(targetHeading);

  unsigned long turnStart = millis();
  unsigned long timeout = 2000; // 2 seconds safety limit to prevent getting stuck
  
  float error = normalizeAngle(targetHeading - yawAngle);

  while (abs(error) > 4.0 && (millis() - turnStart < timeout)) {
    updateYaw();
    error = normalizeAngle(targetHeading - yawAngle);

    if (error > 0) {
      // Spin Left
      analogWrite(LEFT_FWD, 0);           analogWrite(LEFT_REV, TURN_SPEED);
      analogWrite(RIGHT_FWD, TURN_SPEED);  analogWrite(RIGHT_REV, 0);
    } else {
      // Spin Right
      analogWrite(LEFT_FWD, TURN_SPEED); analogWrite(LEFT_REV, 0);
      analogWrite(RIGHT_FWD, 0);          analogWrite(RIGHT_REV, TURN_SPEED);
    }
    delay(5);
  }

  // Active electric brake to halt bot instantly
  activeBrake();
  gyroDelay(150); // Let physical settling happen while integrating yaw

  Serial.print(F("[TURN] Done. Final Yaw = ")); Serial.println(yawAngle);
}


// =============================================================
//  MOTOR HELPERS
// =============================================================
void reverseSmall() {
  analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, FORWARD_SPEED);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, FORWARD_SPEED);
  gyroDelay(500);
  stopMotors();
}

void stopMotors() {
  analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, 0);
}


// =============================================================
//  TCS3200 COLOR
// =============================================================
int readColorPulse(bool s2, bool s3) {
  digitalWrite(TCS_S2, s2 ? HIGH : LOW);
  digitalWrite(TCS_S3, s3 ? HIGH : LOW);
  return (int)pulseIn(TCS_OUT, LOW, 25000);
}

bool isBlackTile() {
  int r = readColorPulse(LOW, LOW);
  int g = readColorPulse(HIGH, HIGH);
  int b = readColorPulse(LOW, HIGH);
  return (r > BLACK_THRESH && g > BLACK_THRESH && b > BLACK_THRESH);
}

bool isBlueTile() {
  int r = readColorPulse(LOW, LOW);
  int b = readColorPulse(LOW, HIGH);
  return (b < r - 40 && b < 150);
}


// =============================================================
//  SERVO MED-KIT DISPENSER (Manual PWM)
//  Pull/Rest Position = 0 deg   (500us pulse)
//  Push/Drop Position = 138 deg (1 push drops 1 kit)
// =============================================================
void servoWrite(int angle) {
  angle = constrain(angle, 0, 180);
  int pulseUs = map(angle, 0, 180, 500, 2500);
  for (int i = 0; i < 35; i++) {
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(SERVO_PIN, LOW);
    delayMicroseconds(20000 - pulseUs);
  }
}

void dispenseKits(int count) {
  for (int i = 0; i < count; i++) {
    servoWrite(138);  // Push kit out (138 deg)
    gyroDelay(400);
    servoWrite(0);    // Pull back to rest (0 deg)
    gyroDelay(400);
  }
}


// =============================================================
//  LED & BUZZER
// =============================================================
void beep(int ms) {
  tone(BUZZER, 2000, ms);
  gyroDelay(ms);
  noTone(BUZZER); // Stop Timer 2 interrupt so software PWM is not corrupted!
}

void blinkLED(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    checkPiCommand(); // Keep listening even while blinking!
    digitalWrite(BLUE_LED, HIGH); gyroDelay(250);
    digitalWrite(BLUE_LED, LOW);  gyroDelay(250);
  }
}

void flashLED(int times, int onMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BLUE_LED, HIGH); gyroDelay(onMs);
    digitalWrite(BLUE_LED, LOW);  gyroDelay(200);
  }
}
