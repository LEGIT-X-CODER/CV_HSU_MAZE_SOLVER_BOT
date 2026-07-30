// ============================================================
//  RESCUE MAZE BOT - ARDUINO UNO FIRMWARE (MPU6500 EDITION)
//
//  HARDWARE:
//    - ToF Sensors: Left=D7, Front=D2, Right=D4
//    - Motors: Left=D5(fwd)/D6(rev), Right=D9(fwd)/D10(rev)
//    - MPU6500 IMU: I2C (A4/A5, Address 0x68)
//    - TCS3200 Color: S0=A1, S1=A2, S2=D12, S3=D11, OUT=D13
//    - Buzzer=D3, Tile LED=A0, Victim LED=A3, Servo=D8
//
//  NAVIGATION & VICTIM WORKFLOW:
//    1. Front obstacle stop threshold: 12 cm.
//    2. Open path threshold: 35 cm.
//    3. Relative Yaw Reset: Before turning, reset yaw to 0.0.
//    4. Blue Tile: Stop 2.5s + Blink LED. 4.0s cooldown buffer.
//    5. Black Tile: Immediate 1.0s reverse with buzzer + 90° turn.
//    6. Asynchronous Victim Buffer: Pi detects H/S/U while driving.
//       Arduino buffers pending victim and drops kits upon reaching
//       the wall before turning, then resets pending victim to none!
//    7. Diagnostic Suite: Triggered by command 'T' from Pi/Serial.
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
#define TILE_LED     A0   // Tile LED on A0
#define VICTIM_LED   A3   // Victim LED on A3
#define SERVO_PIN    8    // Med-kit dispenser servo

#define TCS_S0       A1
#define TCS_S1       A2
#define TCS_S2       12
#define TCS_S3       11
#define TCS_OUT      13

#define MPU_ADDR     0x68

// ---- SPEED & NAVIGATION TUNING PARAMETERS -------------------
int FORWARD_SPEED = 230;   // Driving speed (0-255)
int TURN_SPEED    = 245;   // Turning speed (0-255)

#define WALL_DIST        12   // cm - Stop at wall
#define OPEN_PATH_DIST   35   // cm - Minimum needed for open path
#define BLACK_THRESH    160   // TCS pulse threshold for black tile

const unsigned long BLUE_PAUSE_MS    = 2500; // 2.5s blue tile pause
const unsigned long BLUE_COOLDOWN_MS = 4000; // 4.0s buffer before re-detecting blue
const unsigned long BLACK_REV_MS     = 1000; // 1.0s direct reverse on black

// ---- OBJECTS ------------------------------------------------
VL53L0X tofLeft, tofFront, tofRight;

// ---- GLOBALS ------------------------------------------------
int distL = 200, distF = 200, distR = 200;
bool leftOK  = false;
bool frontOK = false;
bool rightOK = false;

// MPU6500 Gyro & Yaw Variables
float gyroZ_offset  = 0.0;
float yaw           = 0.0;
unsigned long previousTime = 0;
unsigned long lastDistSend  = 0;

// Asynchronous Victim Buffer State
char pendingVictim  = ' ';   // 'H', 'S', 'U', or ' '
bool mazeStarted    = false;
unsigned long blueCooldownUntil = 0;

// Function Prototypes
void stopMotors();
void activeBrake();
void readGyro(int16_t &gx, int16_t &gy, int16_t &gz);
void calibrateGyro();
void updateYaw();
void turnDegrees(float targetAngle);
void handlePendingVictimAtWall();
void runDiagnosticSuite();
void checkPiCommand();
void waitForStart();
void dispenseKits(int count);
void beep(int ms);
void blinkLED(unsigned long ms);
void flashLED(int times, int onMs);


// =============================================================
//  SETUP
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
  delay(500);

  Serial.println(F("\n=============================================="));
  Serial.println(F("   RESCUE MAZE BOT - MPU6500 EDITION"));
  Serial.println(F("=============================================="));

  // Initialize ToF & MPU6500 Gyro
  initToFSensors();
  Serial.println(F("Calibrating MPU6500 Gyro... Keep Still!"));
  calibrateGyro();
  previousTime = micros();

  beep(100); delay(50); beep(100);
  Serial.println(F("[READY] Standby mode. Send 'T' for diagnostics or 'START' to run.\n"));
  
  waitForStart();
}


// =============================================================
//  MAIN NAVIGATION LOOP
// =============================================================
void loop() {
  // ---- Always check for Pi commands (NON-BLOCKING) ----
  checkPiCommand();

  // ---- Update MPU6500 Yaw Angle continuously ----
  updateYaw();

  // ---- Read ToF Distances & Send to Pi ----
  readDistances();
  sendDistanceToPi();

  // ---- FLOOR COLOR CHECK 1: BLACK TILE (Hazard) ----
  // Immediately reverse for 1 sec with buzzer, then turn 90°
  if (isBlackTile()) {
    Serial.println(F("[HAZARD] BLACK TILE! Immediate 1s Reverse + Buzz"));
    
    // Direct Reverse for 1s with Buzzer ON
    tone(BUZZER, 2000);
    analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, FORWARD_SPEED);
    analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, FORWARD_SPEED);
    
    unsigned long revStart = millis();
    while (millis() - revStart < BLACK_REV_MS) {
      updateYaw();
      delay(10);
    }
    noTone(BUZZER);
    stopMotors();

    // Re-read side distances and turn 90° toward open side
    readDistances();
    if (distL >= OPEN_PATH_DIST) {
      turnDegrees(+90.0);
    } else if (distR >= OPEN_PATH_DIST) {
      turnDegrees(-90.0);
    } else {
      turnDegrees(+90.0);
    }
    return;
  }

  // ---- FLOOR COLOR CHECK 2: BLUE TILE (Puddle Checkpoint) ----
  // Stop for 2.5s while blinking LED. Uses 4.0s cooldown buffer.
  if (isBlueTile() && millis() > blueCooldownUntil) {
    Serial.println(F("[TILE] BLUE PUDDLE -> 2.5s Stop + Blink"));
    stopMotors();
    blinkLED(BLUE_PAUSE_MS);
    blueCooldownUntil = millis() + BLUE_COOLDOWN_MS;
    return;
  }

  // ---- WALL AHEAD CHECK (distF <= 12 cm) ----
  if (distF > 0 && distF <= WALL_DIST) {
    stopMotors();
    delay(100);

    // Re-read accurate distances while stopped
    readDistances();
    sendDistanceToPi();

    Serial.print(F("[WALL REACHED] Front=")); Serial.print(distF);
    Serial.print(F("cm | L=")); Serial.print(distL);
    Serial.print(F("cm | R=")); Serial.print(distR); Serial.println(F("cm"));

    // ── STEP 1: Handle Pending Victim BEFORE Turning ─────────────
    if (pendingVictim != ' ') {
      handlePendingVictimAtWall();
    }

    // ── STEP 2: SLRB Turn Decision (Minimum 35 cm for open) ─────
    bool leftOpen  = (distL >= OPEN_PATH_DIST);
    bool rightOpen = (distR >= OPEN_PATH_DIST);

    if (leftOpen && rightOpen) {
      // Both open (>= 35cm): turn toward the side with more open room
      if (distL >= distR) {
        Serial.println(F("-> Both open >=35cm -> Turning LEFT (more space)"));
        turnDegrees(+90.0);
      } else {
        Serial.println(F("-> Both open >=35cm -> Turning RIGHT (more space)"));
        turnDegrees(-90.0);
      }
    } else if (leftOpen) {
      Serial.println(F("-> Turning LEFT (>=35cm open)"));
      turnDegrees(+90.0);
    } else if (rightOpen) {
      Serial.println(F("-> Turning RIGHT (>=35cm open)"));
      turnDegrees(-90.0);
    } else {
      Serial.println(F("-> DEAD END -> Double Left Turn (+90° x2)"));
      turnDegrees(+90.0);
      delay(100);
      turnDegrees(+90.0);
    }
    return;
  }

  // ---- DRIVE FORWARD WITH GYRO STRAIGHT-LINE CORRECTION ----
  float angleError = yaw; // Yaw is reset to 0.0 after every turn
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
  delay(10);
}


// =============================================================
//  MPU6500 GYRO FUNCTIONS (User Verified Integration)
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

void calibrateGyro() {
  long sum = 0;
  for (int i = 0; i < 2000; i++) {
    int16_t gx, gy, gz;
    readGyro(gx, gy, gz);
    sum += gz;
    digitalWrite(TILE_LED, (i / 100) % 2);
    delay(2);
  }
  gyroZ_offset = (sum / 2000.0) / 131.0;
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
//  GYRO TURNING LOGIC (Resets Yaw to 0.0 before turn)
// =============================================================
void turnDegrees(float targetAngle) {
  stopMotors();
  delay(50);

  // Take the yaw value to 0 and turn with respect to it
  yaw = 0.0;
  previousTime = micros();

  Serial.print(F("[TURN] Starting MPU6500 Turn to ")); Serial.print(targetAngle); Serial.println(F("°"));

  unsigned long turnStart = millis();
  unsigned long timeout = 2500; // 2.5s safety limit

  float targetMag = abs(targetAngle) - 3.5; // 3.5° lead-in for motor inertia stop

  while (abs(yaw) < targetMag && (millis() - turnStart < timeout)) {
    updateYaw();

    if (targetAngle > 0) {
      // Spin Left (+90°)
      analogWrite(LEFT_FWD, 0);           analogWrite(LEFT_REV, TURN_SPEED);
      analogWrite(RIGHT_FWD, TURN_SPEED);  analogWrite(RIGHT_REV, 0);
    } else {
      // Spin Right (-90°)
      analogWrite(LEFT_FWD, TURN_SPEED); analogWrite(LEFT_REV, 0);
      analogWrite(RIGHT_FWD, 0);          analogWrite(RIGHT_REV, TURN_SPEED);
    }
    delay(5);
  }

  activeBrake();

  // Reset yaw to 0.0 for straight line driving
  yaw = 0.0;
  previousTime = micros();
  Serial.print(F("[TURN] Complete! Reset Yaw to 0.0°\n"));
}


// =============================================================
//  HANDLE PENDING VICTIM AT THE FORWARD WALL
// =============================================================
void handlePendingVictimAtWall() {
  Serial.print(F("🔥 [WALL VICTIM DEPLOY] Executing Pending Victim: "));
  Serial.println(pendingVictim);

  stopMotors();

  if (pendingVictim == 'H') {
    // 2 Kits + 3 Beeps + 3 LED Flashes
    beep(200); delay(100); beep(200); delay(100); beep(200);
    dispenseKits(2);
    flashLED(3, 300);

  } else if (pendingVictim == 'S') {
    // 1 Kit + 2 Beeps + 2 LED Flashes
    beep(200); delay(100); beep(200);
    dispenseKits(1);
    flashLED(2, 300);

  } else if (pendingVictim == 'U') {
    // 0 Kits + 1 Beep + 1 LED Flash
    beep(200);
    flashLED(1, 500);
  }

  // Send ACK to Pi and reset pending victim back to none
  Serial.print(F("VICTIM_ACK:"));
  Serial.println(pendingVictim);
  
  pendingVictim = ' '; // Cleared back to none!
  delay(300);
}


// =============================================================
//  AUTOMATED DIAGNOSTIC SUITE (Triggered by 'T' command)
// =============================================================
void runDiagnosticSuite() {
  stopMotors();
  Serial.println(F("\n=============================================="));
  Serial.println(F("   STARTING MASTER HARDWARE DIAGNOSTIC SUITE"));
  Serial.println(F("=============================================="));

  // 1. ToF Sensors Stream (3 seconds)
  Serial.println(F("\n[1/4] Testing ToF Sensors (3s Stream)..."));
  unsigned long tofStart = millis();
  while (millis() - tofStart < 3000) {
    readDistances();
    Serial.print(F("  Left="));  Serial.print(distL);
    Serial.print(F("cm | Front=")); Serial.print(distF);
    Serial.print(F("cm | Right=")); Serial.print(distR); Serial.println(F("cm"));
    delay(300);
  }

  // 2. LED & Buzzer Diagnostic (1s each)
  Serial.println(F("\n[2/4] Testing LEDs & Buzzer (1s each)..."));
  Serial.println(F("  --> Tile LED (A0) ON"));
  digitalWrite(TILE_LED, HIGH); delay(1000); digitalWrite(TILE_LED, LOW);

  Serial.println(F("  --> Victim LED (A3) ON"));
  digitalWrite(VICTIM_LED, HIGH); delay(1000); digitalWrite(VICTIM_LED, LOW);

  Serial.println(F("  --> Buzzer (Pin 3) Beep"));
  beep(1000);

  // 3. MPU6500 Gyro Calibration & 5s Live Yaw Stream
  Serial.println(F("\n[3/4] MPU6500 Gyro Calibration & Live Yaw Stream (5s)..."));
  calibrateGyro();
  previousTime = micros();
  unsigned long mpuStart = millis();
  while (millis() - mpuStart < 5000) {
    updateYaw();
    Serial.print(F("  MPU6500 Live Yaw: ")); Serial.print(yaw, 2); Serial.println(F("°"));
    delay(150);
  }

  // 4. 90-Degree Turn Tests
  Serial.println(F("\n[4/4] Executing 90-Degree MPU6500 Self-Test Turns..."));
  Serial.println(F("  --> Left Turn (+90°)"));
  turnDegrees(+90.0); delay(500);

  Serial.println(F("  --> Right Turn (-90°)"));
  turnDegrees(-90.0); delay(500);

  Serial.println(F("\n=============================================="));
  Serial.println(F("   DIAGNOSTIC SUITE COMPLETE! READY FOR MAZE RUN."));
  Serial.println(F("==============================================\n"));

  beep(200); delay(100); beep(200); delay(100); beep(200);
}


// =============================================================
//  CHECK PI SERIAL COMMANDS
// =============================================================
void checkPiCommand() {
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    // Trigger full diagnostic suite if 'T' or 'TEST' is sent
    if (cmd == "T" || cmd.indexOf("TEST") != -1) {
      runDiagnosticSuite();
      return;
    }

    if (cmd.indexOf("STOP") != -1 || cmd.indexOf("RESET") != -1) {
      stopMotors();
      pendingVictim = ' ';
      Serial.println(F("STOP_ACK"));
      beep(100); delay(50); beep(100);
      waitForStart();
      return;
    }

    // Buffer incoming victim signal from Pi
    int idx = cmd.indexOf("VICTIM:");
    if (idx != -1) {
      char v = cmd.charAt(idx + 7);  // 'H', 'S', or 'U'
      if (v == 'H' || v == 'S' || v == 'U') {
        pendingVictim = v;
        Serial.print(F("PI_VICTIM_BUFFERED:")); Serial.println(pendingVictim);
      }
    }
  }
}


// =============================================================
//  WAIT FOR PI START HANDSHAKE
// =============================================================
void waitForStart() {
  stopMotors();
  mazeStarted = false;
  pendingVictim = ' ';
  Serial.println(F("READY"));
  Serial.println(F("Waiting for Pi START or 'T' for Diagnostics..."));

  while (!mazeStarted) {
    if (Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      cmd.toUpperCase();

      if (cmd == "T" || cmd.indexOf("TEST") != -1) {
        runDiagnosticSuite();
        Serial.println(F("READY"));
      } else if (cmd.indexOf("START") != -1) {
        mazeStarted = true;
        yaw = 0.0;
        previousTime = micros();
        Serial.println(F("START received — maze running!"));
        beep(200); delay(100); beep(200);
      }
    }
    digitalWrite(TILE_LED, (millis() / 500) % 2);
    delay(10);
  }
  digitalWrite(TILE_LED, LOW);
}


// =============================================================
//  SEND FRONT DISTANCE TO PI
// =============================================================
void sendDistanceToPi() {
  if (millis() - lastDistSend >= 200) {
    Serial.print(F("D:")); Serial.println(distF);
    lastDistSend = millis();
  }
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
//  TCS3200 COLOR SENSOR
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
    checkPiCommand();
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
