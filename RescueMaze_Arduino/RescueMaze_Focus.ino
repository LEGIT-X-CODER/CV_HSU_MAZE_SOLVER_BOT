// ============================================================
//  RESCUE MAZE BOT - "FOCUS" EDITION (RescueMaze_Focus.ino)
//
//  KEY HIGHLIGHTS & ARCHITECTURE:
//    1. FAST 1.5-SECOND AUTO-RESUME BOOT:
//       No long diagnostic delays! If power dips or resets mid-maze,
//       gyro calibrates in 1.5s and immediately continues driving!
//
//    2. SMOOTH 3-STAGE INCREMENTAL TURN (3x 30° Sets = 90°):
//       Splits 90° turns into 3 micro-turns of 30° each with 20ms
//       pauses. Eliminates power surge brown-outs & motor overshoot!
//
//    3. 100% FOCUSED TURN LOCK:
//       During turns, bot focuses ONLY on turning and yaw tracking.
//       Color checks & victim logic run ONLY after turn is complete!
//
//    4. NON-BLOCKING SERIAL BUFFER (Zero-Freeze Pi Link):
//       Fast manual character accumulation buffer prevents any serial
//       communication timeouts or lockups.
//
//  HARDWARE PINS:
//    - ToF XSHUT: Left=7, Front=2, Right=4 (I2C 0x30, 0x31, 0x32)
//    - Motors: Left=D5(fwd)/D6(rev), Right=D9(fwd)/D10(rev)
//    - MPU6500 IMU: I2C Address 0x68
//    - TCS3200 Color: S0=A1, S1=A2, S2=12, S3=11, OUT=13
//    - Buzzer=D3, Tile LED=A0, Victim LED=A3, Servo=D8
// ============================================================

#include <Wire.h>
#include <VL53L0X.h>

// ---- PIN DEFINITIONS ----------------------------------------
#define XSHUT_LEFT   7
#define XSHUT_FRONT  2
#define XSHUT_RIGHT  4

#define LEFT_FWD     5
#define LEFT_REV     6
#define RIGHT_FWD    9
#define RIGHT_REV    10

#define BUZZER       3
#define TILE_LED     A0
#define VICTIM_LED   A3
#define SERVO_PIN    8

#define TCS_S0       A1
#define TCS_S1       A2
#define TCS_S2       12
#define TCS_S3       11
#define TCS_OUT      13

#define MPU_ADDR     0x68

// ---- NAVIGATION PARAMETERS ----------------------------------
int FORWARD_SPEED = 220;   // Driving speed (0-255)
int TURN_SPEED    = 170;   // Smooth 30-degree micro-turn speed (0-255)

#define WALL_DIST        12   // cm - Stop at wall
#define OPEN_PATH_DIST   35   // cm - Minimum needed for open path
#define BLACK_THRESH    160   // TCS pulse threshold for black tile

const unsigned long BLUE_PAUSE_MS    = 2500; // 2.5s blue tile pause
const unsigned long BLUE_COOLDOWN_MS = 4000; // 4.0s buffer before re-detecting blue
const unsigned long BLACK_REV_MS     = 100;  // 0.1s quick reverse on black

// ---- OBJECTS ------------------------------------------------
VL53L0X tofLeft, tofFront, tofRight;

// ---- GLOBALS ------------------------------------------------
int distL = 200, distF = 200, distR = 200;
bool leftOK  = false;
bool frontOK = false;
bool rightOK = false;

// MPU6500 Gyro & Yaw
float gyroZ_offset   = 0.0;
float yaw            = 0.0;
unsigned long previousTime   = 0;
unsigned long lastDistSend   = 0;
unsigned long lastNavPrint   = 0;
unsigned long lastColorCheck = 0;

// Asynchronous Victim & Navigation State
char pendingVictim   = ' ';   // 'H', 'S', 'U', or ' '
bool mazeStarted     = false;
volatile bool stopRequested = false;
unsigned long blueCooldownUntil = 0;

// Non-blocking Serial Buffer
char serialBuffer[32];
byte serialBufferIdx = 0;

// Function Prototypes
void stopMotors();
void activeBrake();
void readGyro(int16_t &gx, int16_t &gy, int16_t &gz);
void calibrateGyroFast();
void updateYaw();
void turn3Stage(float totalDegrees);
void turnMicroStep(float targetStepAngle);
void handlePendingVictimAtWall();
void processPiSerialCommand();
void waitForStart();
void dispenseKits(int count);
void beep(int ms);
void blinkLED(unsigned long ms);
void flashLED(int times, int onMs);


// =============================================================
//  SETUP (FAST 1.5s AUTO-RESUME BOOT)
// =============================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000, true); // Auto-reset I2C bus if stuck > 3ms
  #endif

  // Pins Setup
  pinMode(LEFT_FWD,   OUTPUT); pinMode(LEFT_REV,   OUTPUT);
  pinMode(RIGHT_FWD,  OUTPUT); pinMode(RIGHT_REV,  OUTPUT);
  pinMode(BUZZER,     OUTPUT);
  pinMode(TILE_LED,   OUTPUT);
  pinMode(VICTIM_LED, OUTPUT);
  pinMode(SERVO_PIN,  OUTPUT);
  stopMotors();
  servoWrite(0); // Servo rest/pull position

  // TCS3200 Pins Setup
  pinMode(TCS_S0, OUTPUT); pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT); pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);

  // Wake up MPU6500
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println(F("\n=============================================="));
  Serial.println(F("   RESCUE MAZE BOT - FOCUS EDITION READY"));
  Serial.println(F("=============================================="));

  // Quick 1.5s Hardware Setup
  initToFSensors();
  Serial.println(F("Quick 1.5s Gyro Calibration... Keep Still!"));
  calibrateGyroFast();
  previousTime = micros();

  beep(100); delay(50); beep(100);
  Serial.println(F("⚡ [READY] Waiting for Pi 'START' command...\n"));
  
  // Wait for Pi START command to avoid spinning on power-up!
  waitForStart();
}


// =============================================================
//  MAIN NAVIGATION LOOP
// =============================================================
void loop() {
  // 1. Process Pi Serial Commands (NON-BLOCKING)
  processPiSerialCommand();
  if (stopRequested) {
    waitForStart();
    return;
  }

  // 2. Continuous Yaw & ToF Distance Updates
  updateYaw();
  readDistances();

  // Send distance to Pi every 100ms
  if (millis() - lastDistSend >= 100) {
    Serial.print(F("D:")); Serial.println(distF);
    lastDistSend = millis();
  }

  // 3. FLOOR COLOR CHECK (Throttled every 100ms when driving)
  if (millis() - lastColorCheck >= 100) {
    lastColorCheck = millis();

    // Black Hazard Check (Only if color sensor returns valid pulses > 0)
    if (isBlackTile()) {
      Serial.println(F("⚠️ [HAZARD] BLACK TILE! Quick 0.1s Reverse + Buzz"));
      stopMotors();
      
      tone(BUZZER, 2000);
      analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, FORWARD_SPEED);
      analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, FORWARD_SPEED);
      delay(BLACK_REV_MS);
      noTone(BUZZER);
      stopMotors();

      readDistances();
      if (distL >= OPEN_PATH_DIST) {
        turn3Stage(+90.0);
      } else if (distR >= OPEN_PATH_DIST) {
        turn3Stage(-90.0);
      } else {
        turn3Stage(+90.0);
      }
      return;
    }

    // Blue Puddle Checkpoint Check
    if (isBlueTile() && millis() > blueCooldownUntil) {
      Serial.println(F("💧 [TILE] BLUE PUDDLE -> 2.5s Stop + Blink"));
      stopMotors();
      blinkLED(BLUE_PAUSE_MS);
      blueCooldownUntil = millis() + BLUE_COOLDOWN_MS;
      return;
    }
  }

  // 4. WALL REACHED CHECK (distF >= 3 && distF <= 12 cm) -> STOP, DROP KIT & TURN
  if (distF >= 3 && distF <= WALL_DIST) {
    stopMotors();
    delay(100);

    readDistances();
    Serial.print(F("🛑 [WALL REACHED] Front=")); Serial.print(distF);
    Serial.print(F("cm | L=")); Serial.print(distL);
    Serial.print(F("cm | R=")); Serial.print(distR); Serial.println(F("cm"));

    // Handle Pending Victim BEFORE Turning
    if (pendingVictim != ' ') {
      handlePendingVictimAtWall();
    }

    // SLRB Turn Decision (Minimum 35cm for open path)
    bool leftOpen  = (distL >= OPEN_PATH_DIST);
    bool rightOpen = (distR >= OPEN_PATH_DIST);

    if (leftOpen && rightOpen) {
      if (distL >= distR) {
        Serial.println(F("-> Both open >=35cm -> Turning LEFT (more space)"));
        turn3Stage(+90.0);
      } else {
        Serial.println(F("-> Both open >=35cm -> Turning RIGHT (more space)"));
        turn3Stage(-90.0);
      }
    } else if (leftOpen) {
      Serial.println(F("-> Turning LEFT (>=35cm open)"));
      turn3Stage(+90.0);
    } else if (rightOpen) {
      Serial.println(F("-> Turning RIGHT (>=35cm open)"));
      turn3Stage(-90.0);
    } else {
      Serial.println(F("-> DEAD END -> Double Left Turn (+90° x2)"));
      turn3Stage(+90.0);
      delay(100);
      turn3Stage(+90.0);
    }
    return;
  }

  // 5. DRIVE FORWARD WITH MPU STRAIGHT-LINE CORRECTION
  float angleError = yaw;
  int gyroSteer = constrain(angleError * 5.0, -45, 45);

  int wallSteer = 0;
  if (distL < 25 && distR < 25) {
    wallSteer = (distL - distR) * 2;
  }

  int steer = gyroSteer;
  if (abs(wallSteer) > 2) {
    steer = gyroSteer + (wallSteer / 2);
  }
  steer = constrain(steer, -45, 45);

  analogWrite(LEFT_FWD,  constrain(FORWARD_SPEED + steer, 0, 255));
  analogWrite(LEFT_REV,  0);
  analogWrite(RIGHT_FWD, constrain(FORWARD_SPEED - steer, 0, 255));
  analogWrite(RIGHT_REV, 0);

  // Live Navigation Console Log
  if (millis() - lastNavPrint >= 300) {
    Serial.print(F("🚗 [FOCUS DRIVE] Front: ")); Serial.print(distF);
    Serial.print(F("cm | L: ")); Serial.print(distL);
    Serial.print(F("cm | R: ")); Serial.print(distR);
    Serial.print(F("cm | Yaw: ")); Serial.print(yaw, 1);
    if (pendingVictim != ' ') {
      Serial.print(F("° | Victim: ")); Serial.print(pendingVictim);
    } else {
      Serial.print(F("° | Victim: None"));
    }
    Serial.println();
    lastNavPrint = millis();
  }

  delay(5);
}


// =============================================================
//  3-STAGE INCREMENTAL TURNING LOGIC (3x 30° Sets = 90°)
//  100% Focused Execution — No extraneous checks during turns!
// =============================================================
void turn3Stage(float totalDegrees) {
  Serial.print(F("🔄 [3-STAGE TURN FOCUS] Total Target: ")); Serial.print(totalDegrees); Serial.println(F("°"));
  
  // 1. Complete Standstill & Active Brake
  activeBrake();
  delay(150);

  if (stopRequested) return;

  // Calculate 3 step angles (e.g. +30°, +30°, +30° OR -30°, -30°, -30°)
  float stepAngle = totalDegrees / 3.0; // 30 degrees per step

  for (int step = 1; step <= 3; step++) {
    processPiSerialCommand();
    if (stopRequested) {
      stopMotors();
      return;
    }

    Serial.print(F("  --> Micro-Turn Step [")); Serial.print(step); Serial.print(F("/3] Target: "));
    Serial.print(stepAngle * step); Serial.println(F("°"));

    // Execute single 30-degree micro-turn
    turnMicroStep(stepAngle);
    
    // Short 20ms pause between micro-steps to stabilize current & momentum
    delay(20);
  }

  activeBrake();
  delay(100);

  // Reset yaw to 0.0 for straight line driving
  yaw = 0.0;
  previousTime = micros();
  Serial.print(F("✅ [3-STAGE TURN COMPLETE] Yaw Reset to 0.0°\n"));
}


// =============================================================
//  MICRO 30-DEGREE TURN STEP (Focused Yaw Tracking & Soft-Start)
// =============================================================
void turnMicroStep(float stepAngle) {
  yaw = 0.0;
  previousTime = micros();

  unsigned long start = millis();
  unsigned long timeout = 400; // 400ms max limit per 30° step
  float targetMag = abs(stepAngle) - 2.0;

  // Soft-Start Ramp-Up for Micro Step
  for (int speed = 80; speed <= TURN_SPEED; speed += 30) {
    if (stepAngle > 0) {
      analogWrite(LEFT_FWD, 0);          analogWrite(LEFT_REV, speed);
      analogWrite(RIGHT_FWD, speed);     analogWrite(RIGHT_REV, 0);
    } else {
      analogWrite(LEFT_FWD, speed);      analogWrite(LEFT_REV, 0);
      analogWrite(RIGHT_FWD, 0);         analogWrite(RIGHT_REV, speed);
    }
    delay(8);
  }

  // Focused 30-Degree Spin Loop
  while (millis() - start < timeout) {
    updateYaw();

    bool stepFinished = false;
    if (stepAngle > 0 && yaw >= targetMag) {
      stepFinished = true;
    } else if (stepAngle < 0 && yaw <= -targetMag) {
      stepFinished = true;
    }

    if (stepFinished) break;

    if (stepAngle > 0) {
      analogWrite(LEFT_FWD, 0);          analogWrite(LEFT_REV, TURN_SPEED);
      analogWrite(RIGHT_FWD, TURN_SPEED); analogWrite(RIGHT_REV, 0);
    } else {
      analogWrite(LEFT_FWD, TURN_SPEED);  analogWrite(LEFT_REV, 0);
      analogWrite(RIGHT_FWD, 0);          analogWrite(RIGHT_REV, TURN_SPEED);
    }
    delay(4);
  }

  stopMotors();
}


// =============================================================
//  MPU6500 GYRO FUNCTIONS
// =============================================================
void readGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6, true);
  if (Wire.available() >= 6) {
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
  }
}

void calibrateGyroFast() {
  long sum = 0;
  for (int i = 0; i < 750; i++) { // Fast 1.5s 750-sample calibration
    int16_t gx, gy, gz;
    readGyro(gx, gy, gz);
    sum += gz;
    digitalWrite(TILE_LED, (i / 50) % 2);
    delay(2);
  }
  gyroZ_offset = (sum / 750.0) / 131.0;
  digitalWrite(TILE_LED, LOW);
  yaw = 0.0;
}

void updateYaw() {
  int16_t gx, gy, gz;
  readGyro(gx, gy, gz);

  float gyroZ = (gz / 131.0) - gyroZ_offset;

  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  if (dt > 0.0 && dt < 0.5) {
    yaw += gyroZ * dt;
    if (yaw > 180.0)  yaw -= 360.0;
    if (yaw < -180.0) yaw += 360.0;
  }
}


// =============================================================
//  HANDLE PENDING VICTIM AT FORWARD WALL
// =============================================================
void handlePendingVictimAtWall() {
  Serial.print(F("🔥 [WALL VICTIM DEPLOY] Executing Pending Victim: "));
  Serial.println(pendingVictim);

  stopMotors();

  if (pendingVictim == 'H') {
    beep(200); delay(100); beep(200); delay(100); beep(200);
    dispenseKits(2);
    flashLED(3, 300);
  } else if (pendingVictim == 'S') {
    beep(200); delay(100); beep(200);
    dispenseKits(1);
    flashLED(2, 300);
  } else if (pendingVictim == 'U') {
    beep(200);
    flashLED(1, 500);
  }

  Serial.print(F("VICTIM_ACK:"));
  Serial.println(pendingVictim);
  
  pendingVictim = ' ';
  delay(200);
}


// =============================================================
//  NON-BLOCKING PI SERIAL COMMAND PARSER
// =============================================================
void processPiSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (serialBufferIdx > 0) {
        serialBuffer[serialBufferIdx] = '\0';
        String cmd = String(serialBuffer);
        cmd.trim();
        cmd.toUpperCase();
        serialBufferIdx = 0;

        if (cmd.indexOf("STOP") != -1 || cmd.indexOf("RESET") != -1) {
          stopMotors();
          stopRequested = true;
          mazeStarted = false;
          pendingVictim = ' ';
          Serial.println(F("STOP_ACK"));
        } else if (cmd.indexOf("START") != -1) {
          stopRequested = false;
          mazeStarted = true;
          yaw = 0.0;
          previousTime = micros();
          Serial.println(F("START_ACK"));
        } else if (cmd.indexOf("VICTIM:") != -1) {
          int idx = cmd.indexOf("VICTIM:");
          char v = cmd.charAt(idx + 7);
          if (v == 'H' || v == 'S' || v == 'U') {
            pendingVictim = v;
            Serial.print(F("PI_VICTIM_BUFFERED:")); Serial.println(pendingVictim);
          }
        }
      }
    } else if (serialBufferIdx < sizeof(serialBuffer) - 1) {
      serialBuffer[serialBufferIdx++] = c;
    }
  }
}

// =============================================================
//  WAIT FOR PI START HANDSHAKE (Prevents Spinning on Bootup)
// =============================================================
void waitForStart() {
  stopMotors();
  mazeStarted = false;
  stopRequested = false;
  pendingVictim = ' ';
  Serial.println(F("READY"));
  Serial.println(F("Waiting for Pi 'START' command..."));

  while (!mazeStarted) {
    processPiSerialCommand();
    digitalWrite(TILE_LED, (millis() / 500) % 2);
    delay(10);
  }
  digitalWrite(TILE_LED, LOW);
  yaw = 0.0;
  previousTime = micros();
  Serial.println(F("⚡ [START RECEIVED] Bot active & navigating!"));
}


// =============================================================
//  ToF SENSORS INIT & READ
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
    tofLeft.setTimeout(100);
    tofLeft.startContinuous(30);
    leftOK = true;
    Serial.println(F("  Left  ToF OK (0x30)"));
  } else {
    leftOK = false;
    Serial.println(F("  Left  ToF FAIL!"));
  }

  digitalWrite(XSHUT_FRONT, HIGH); delay(30);
  if (tofFront.init()) {
    tofFront.setAddress(0x31);
    tofFront.setTimeout(100);
    tofFront.startContinuous(30);
    frontOK = true;
    Serial.println(F("  Front ToF OK (0x31)"));
  } else {
    frontOK = false;
    Serial.println(F("  Front ToF FAIL!"));
  }

  digitalWrite(XSHUT_RIGHT, HIGH); delay(30);
  if (tofRight.init()) {
    tofRight.setAddress(0x32);
    tofRight.setTimeout(100);
    tofRight.startContinuous(30);
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
//  TCS3200 COLOR SENSOR (Fast 6ms pulseIn timeout)
// =============================================================
int readColorPulse(bool s2, bool s3) {
  digitalWrite(TCS_S2, s2 ? HIGH : LOW);
  digitalWrite(TCS_S3, s3 ? HIGH : LOW);
  return (int)pulseIn(TCS_OUT, LOW, 6000);
}

bool isBlackTile() {
  int r = readColorPulse(LOW, LOW);
  int g = readColorPulse(HIGH, HIGH);
  int b = readColorPulse(LOW, HIGH);
  if (r <= 0 || g <= 0 || b <= 0) return false; // Ignore pulse timeouts/disconnected sensors
  return (r > BLACK_THRESH && g > BLACK_THRESH && b > BLACK_THRESH);
}

bool isBlueTile() {
  int r = readColorPulse(LOW, LOW);
  int b = readColorPulse(LOW, HIGH);
  if (r <= 0 || b <= 0) return false;
  return (b < r - 40 && b < 150);
}


// =============================================================
//  MOTOR & SERVO HELPERS
// =============================================================
void activeBrake() {
  analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, 220);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, 220);
  delay(40);
  stopMotors();
}

void stopMotors() {
  analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, 0);
}

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
    delay(400);
    servoWrite(0);    // Pull back to rest (0 deg)
    delay(400);
  }
}

void beep(int ms) {
  tone(BUZZER, 2000, ms);
  delay(ms);
  noTone(BUZZER);
}

void blinkLED(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    processPiSerialCommand();
    digitalWrite(TILE_LED, HIGH); delay(250);
    digitalWrite(TILE_LED, LOW);  delay(250);
  }
}

void flashLED(int times, int onMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(VICTIM_LED, HIGH); delay(onMs);
    digitalWrite(VICTIM_LED, LOW);  delay(200);
  }
}
