// ============================================================
//  ARDUINO ALL-IN-ONE INTERACTIVE TEST SKETCH
//  
//  SERIAL COMMANDS (Open Serial Monitor at 115200 baud):
//    't' - Test ToF Distance Sensors (Left, Front, Right)
//    'm' - Test Motors (all movements with sound/LED)
//    'i' - Test MPU6500 IMU Angle (Live Yaw Angle in DEGREES)
//    'b' - Test Buzzer
//    'l' - Test Blue LED (A0)
//    'c' - Test TCS3200 Color Sensor (R, G, B & Tile Detection)
//    's' - Test Med-Kit Dispenser Servo (Pin 8 Sweep & Drop)
//    'z' - Test Precise Turns (Left +90°, Right -90°, -180° U-Turn)
//
//  PIN MAP:
//    ToF XSHUT: Left=D7, Front=D2, Right=D4 | I2C = A4/A5
//    MPU6500: I2C (A4/A5) at 0x68
//    Motors: Left=D5(fwd)/D6(rev), Right=D9(fwd)/D10(rev)
//    Buzzer=D3, Blue LED=A0, Servo=D8
//    TCS3200: S0=A1, S1=A2, S2=D12, S3=D11, OUT=D13
//
//  Library needed: VL53L0X by Pololu
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
#define BLUE_LED     A0
#define SERVO_PIN    8    // Servo on Pin 8

#define TCS_S0       A1
#define TCS_S1       A2
#define TCS_S2       12
#define TCS_S3       11
#define TCS_OUT      13

// =============================================================
//  MOTOR SPEED CONTROLS (Change these values from 0 to 255)
// =============================================================
int FORWARD_SPEED = 200;   // Straight driving motor speed (0-255)
int TURN_SPEED    = 220;   // Turning motor speed for 90/180 turns (0-255)

const int MPU_ADDR = 0x68;

// ---- SENSOR OBJECTS & GLOBALS -------------------------------
VL53L0X tofLeft, tofFront, tofRight;
bool tofL_OK = false, tofF_OK = false, tofR_OK = false;
bool mpu_OK = false;

float gyroZOffset = 0.0;
float currentYawAngle = 0.0;
unsigned long lastGyroMicros = 0;

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Pin Configurations
  pinMode(LEFT_FWD,  OUTPUT); pinMode(LEFT_REV,  OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT); pinMode(RIGHT_REV, OUTPUT);
  pinMode(BUZZER,    OUTPUT);
  pinMode(BLUE_LED,  OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  stopMotors();

  // Servo start in rest/pull position (0 deg)
  servoWrite(0);

  // TCS Pins
  pinMode(TCS_S0, OUTPUT); pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT); pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  digitalWrite(TCS_S0, HIGH); digitalWrite(TCS_S1, LOW);

  // Init ToFs
  initToFSensors();

  // Init MPU
  initMPU();

  printMenu();
}

// =============================================================
//  MAIN LOOP - Serial Command Parser
// =============================================================
void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    Serial.print(F("\n>>> EXECUTING COMMAND: '"));
    Serial.print(input);
    Serial.println(F("'"));

    char cmd = input.charAt(0);

    // Custom Servo Angle handling: e.g., 's0', 's90', 's180', 's45'
    if ((cmd == 's' || cmd == 'S') && input.length() > 1) {
      int targetAngle = input.substring(1).toInt();
      targetAngle = constrain(targetAngle, 0, 180);
      Serial.print(F(">>> Moving Servo (Pin 8) to Angle: "));
      Serial.print(targetAngle);
      Serial.println(F(" deg"));
      servoWrite(targetAngle);
    }
    else {
      switch (cmd) {
        case 't': case 'T': testToF(); break;
        case 'm': case 'M': testMotors(); break;
        case 'i': case 'I': testIMUAngle(); break;
        case 'b': case 'B': testBuzzer(); break;
        case 'l': case 'L': testLED(); break;
        case 'c': case 'C': testColorSensor(); break;
        case 's': case 'S': testServo(); break;
        case 'z': case 'Z': testTurning(); break;
        case '?': case 'h': case 'H': printMenu(); break;
        default:
          Serial.println(F("Unknown command! Type 't','m','i','b','l','c','s','s90','s180', or 'z'."));
          break;
      }
    }

    printMenuPrompt();
  }
}

// =============================================================
//  MENU PRINTING
// =============================================================
void printMenu() {
  Serial.println(F("\n=================================================="));
  Serial.println(F("    RESCUE MAZE BOT - INTERACTIVE TEST MENU      "));
  Serial.println(F("=================================================="));
  Serial.println(F("  [t] - Test ToF Distance Sensors (Left, Front, Right)"));
  Serial.println(F("  [m] - Test Motors (Forward, Reverse, Spins + Indications)"));
  Serial.println(F("  [i] - Test IMU Live Angle (Shows actual Yaw Angle in deg)"));
  Serial.println(F("  [b] - Test Buzzer (Beep tones)"));
  Serial.println(F("  [l] - Test Blue LED (A0)"));
  Serial.println(F("  [c] - Test TCS3200 Color Sensor (R, G, B & Tile Type)"));
  Serial.println(F("  [s] - Test Servo Dispenser (Pin 8 Sweep & Kit Drop)"));
  Serial.println(F("  [z] - Test Turning (Left +90 deg, Right -90 deg, -180 deg)"));
  Serial.println(F("=================================================="));
  printMenuPrompt();
}

void printMenuPrompt() {
  Serial.print(F("\nEnter command (t/m/i/b/l/c/s/z): "));
}

// =============================================================
//  COMMAND 't': TEST ToF SENSORS
// =============================================================
void testToF() {
  Serial.println(F("--- Testing ToF Sensors (20 Readings) ---"));
  for (int i = 1; i <= 20; i++) {
    int distL = readDistance(tofLeft,  tofL_OK);
    int distF = readDistance(tofFront, tofF_OK);
    int distR = readDistance(tofRight, tofR_OK);

    Serial.print(F("#")); Serial.print(i);
    Serial.print(F("\tLeft: "));  printDistVal(distL);
    Serial.print(F("\tFront: ")); printDistVal(distF);
    Serial.print(F("\tRight: ")); printDistVal(distR);
    Serial.println();
    delay(250);
  }
  Serial.println(F(">>> ToF Sensor Test Complete."));
}

int readDistance(VL53L0X &sensor, bool isOK) {
  if (!isOK) return -1;
  int mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred() || mm > 2000) return 200;
  return mm / 10; // Convert to cm
}

void printDistVal(int cm) {
  if (cm < 0) Serial.print(F("FAIL"));
  else if (cm >= 200) Serial.print(F(">200cm"));
  else { Serial.print(cm); Serial.print(F("cm")); }
}

// =============================================================
//  COMMAND 'm': TEST MOTORS WITH INDICATION
// =============================================================
void testMotors() {
  Serial.println(F("--- Motor Test with Audio/Visual Indications ---"));
  
  // 1. Left Motor Forward
  Serial.println(F("1/6 Left Motor FORWARD (1 beep, LED solid)"));
  soundIndicate(1, true);
  analogWrite(LEFT_FWD, FORWARD_SPEED); analogWrite(LEFT_REV, 0);
  delay(1200); stopMotors(); delay(500);

  // 2. Left Motor Reverse
  Serial.println(F("2/6 Left Motor REVERSE (1 beep, LED blink)"));
  soundIndicate(1, false);
  analogWrite(LEFT_FWD, 0); analogWrite(LEFT_REV, FORWARD_SPEED);
  delay(1200); stopMotors(); delay(500);

  // 3. Right Motor Forward
  Serial.println(F("3/6 Right Motor FORWARD (2 beeps, LED solid)"));
  soundIndicate(2, true);
  analogWrite(RIGHT_FWD, FORWARD_SPEED); analogWrite(RIGHT_REV, 0);
  delay(1200); stopMotors(); delay(500);

  // 4. Right Motor Reverse
  Serial.println(F("4/6 Right Motor REVERSE (2 beeps, LED blink)"));
  soundIndicate(2, false);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, FORWARD_SPEED);
  delay(1200); stopMotors(); delay(500);

  // 5. Both Forward
  Serial.println(F("5/6 Both Motors FORWARD (3 beeps, LED solid)"));
  soundIndicate(3, true);
  analogWrite(LEFT_FWD, FORWARD_SPEED);  analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, FORWARD_SPEED); analogWrite(RIGHT_REV, 0);
  delay(1200); stopMotors(); delay(500);

  // 6. Both Reverse
  Serial.println(F("6/6 Both Motors REVERSE (3 beeps, LED blink)"));
  soundIndicate(3, false);
  analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, FORWARD_SPEED);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, FORWARD_SPEED);
  delay(1200); stopMotors(); delay(500);

  Serial.println(F(">>> Motor Test Complete."));
}

void soundIndicate(int beeps, bool solidLED) {
  for (int i = 0; i < beeps; i++) {
    tone(BUZZER, 2000, 100);
    digitalWrite(BLUE_LED, HIGH);
    delay(150);
    digitalWrite(BLUE_LED, LOW);
    delay(100);
  }
  digitalWrite(BLUE_LED, solidLED ? HIGH : LOW);
  delay(200);
}

// =============================================================
//  COMMAND 'i': TEST IMU ANGLE (Live Yaw Angle Tracking)
// =============================================================
void testIMUAngle() {
  if (!mpu_OK) {
    Serial.println(F("ERROR: MPU6500 not connected or failed initialization!"));
    return;
  }

  Serial.println(F("--- Calibrating Gyro Offset (KEEP ROBOT STILL for 3 sec)... ---"));
  calibrateGyroOffset(3000);
  Serial.print(F("Calibrated Gyro Offset: ")); Serial.println(gyroZOffset);

  Serial.println(F("\n--- Live Yaw Angle Tracking (15 Seconds) ---"));
  Serial.println(F("Rotate the bot by hand to see the ANGLE IN DEGREES update!"));

  currentYawAngle = 0.0;
  lastGyroMicros = micros();
  unsigned long testStart = millis();

  while (millis() - testStart < 15000) {
    updateYawAngle();
    
    // Print every 200ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
      lastPrint = millis();
      Serial.print(F("Current Angle: "));
      Serial.print(currentYawAngle, 2);
      Serial.println(F(" deg"));
    }
  }
  Serial.println(F(">>> Live IMU Angle Test Complete."));
}

void calibrateGyroOffset(unsigned long durationMs) {
  long sum = 0;
  int count = 0;
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    sum += readRawGyroZ();
    count++;
    digitalWrite(BLUE_LED, (millis() / 100) % 2);
    delay(5);
  }
  digitalWrite(BLUE_LED, LOW);
  if (count > 0) gyroZOffset = (float)sum / (float)count;
}

int16_t readRawGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return ((int16_t)Wire.read() << 8) | Wire.read();
}

void updateYawAngle() {
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastGyroMicros) / 1000000.0;
  lastGyroMicros = nowMicros;

  int16_t rawZ = readRawGyroZ();
  // 131 LSB/(deg/sec) sensitivity for +/- 250 dps range
  float dps = (rawZ - gyroZOffset) / 131.0;
  currentYawAngle += dps * dt;
}

// =============================================================
//  COMMAND 'b': TEST BUZZER
// =============================================================
void testBuzzer() {
  Serial.println(F("--- Testing Buzzer Tones ---"));
  tone(BUZZER, 1000, 150); delay(200);
  tone(BUZZER, 1500, 150); delay(200);
  tone(BUZZER, 2000, 150); delay(200);
  tone(BUZZER, 2500, 300); delay(350);
  Serial.println(F(">>> Buzzer Test Complete."));
}

// =============================================================
//  COMMAND 'l': TEST BLUE LED
// =============================================================
void testLED() {
  Serial.println(F("--- Testing Blue LED (A0) ---"));
  Serial.println(F("Blinking 5 times..."));
  for (int i = 0; i < 5; i++) {
    digitalWrite(BLUE_LED, HIGH); delay(200);
    digitalWrite(BLUE_LED, LOW);  delay(200);
  }
  Serial.println(F(">>> LED Test Complete."));
}

// =============================================================
//  COMMAND 'c': TEST TCS3200 COLOR SENSOR
// =============================================================
void testColorSensor() {
  Serial.println(F("--- TCS3200 Color Sensor Test (20 Readings) ---"));
  Serial.println(F("Place sensor over Black / Blue / White floor tiles to test!"));

  for (int i = 1; i <= 20; i++) {
    int r = readColorPulse(LOW,  LOW);   // Red
    int g = readColorPulse(HIGH, HIGH);  // Green
    int b = readColorPulse(LOW,  HIGH);  // Blue

    Serial.print(F("#")); Serial.print(i);
    Serial.print(F("\tR: ")); Serial.print(r);
    Serial.print(F("\tG: ")); Serial.print(g);
    Serial.print(F("\tB: ")); Serial.print(b);

    if (r > 160 && g > 160 && b > 160) {
      Serial.print(F("\t -> [ BLACK HAZARD TILE ]"));
    } else if (b < r - 40 && b < 150) {
      Serial.print(F("\t -> [ BLUE PUDDLE TILE ]"));
    } else if (r < 100 && g < 100 && b < 100) {
      Serial.print(F("\t -> [ WHITE / SILVER TILE ]"));
    } else {
      Serial.print(F("\t -> [ NORMAL TILE ]"));
    }
    Serial.println();
    delay(300);
  }
  Serial.println(F(">>> TCS3200 Color Sensor Test Complete."));
}

int readColorPulse(bool s2, bool s3) {
  digitalWrite(TCS_S2, s2 ? HIGH : LOW);
  digitalWrite(TCS_S3, s3 ? HIGH : LOW);
  return (int)pulseIn(TCS_OUT, LOW, 25000);
}

// =============================================================
//  COMMAND 's': TEST SERVO DISPENSER (PIN 8)
// =============================================================
void testServo() {
  Serial.println(F("--- Testing Servo Med-Kit Dispenser (Pin 8) ---"));
  Serial.println(F("1. Moving to PUSH position (138 deg)..."));
  servoWrite(138);
  delay(1000);

  Serial.println(F("2. Moving to PULL / REST position (0 deg)..."));
  servoWrite(0);
  delay(1000);

  Serial.println(F("3. Simulating 2 Med-Kit drops (1 push = 1 kit drop)..."));
  for (int i = 1; i <= 2; i++) {
    Serial.print(F("   Dropping kit #")); Serial.println(i);
    tone(BUZZER, 2000, 100);
    servoWrite(138);   // Push kit out (138 deg)
    delay(400);
    servoWrite(0);     // Pull back to rest (0 deg)
    delay(400);
  }
  Serial.println(F(">>> Servo Dispenser Test Complete."));
}

void servoWrite(int angle) {
  // Manual PWM pulse:
  // angle 0   = 500us  (PULL / REST POSITION)
  // angle 138 = 2033us (PUSH POSITION - 1 PUSH DROPS 1 KIT)
  angle = constrain(angle, 0, 180);
  int pulseUs = map(angle, 0, 180, 500, 2500);
  for (int i = 0; i < 35; i++) {
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(SERVO_PIN, LOW);
    delayMicroseconds(20000 - pulseUs);
  }
}

// =============================================================
//  COMMAND 'z': TEST PRECISE TURNS (MPU Closed-Loop)
//  Sign Convention: Left Turn = Positive (+), Right Turn = Negative (-)
// =============================================================
void testTurning() {
  if (!mpu_OK) {
    Serial.println(F("ERROR: MPU6500 required for turns!"));
    return;
  }

  Serial.println(F("--- Calibrating Gyro Offset (KEEP STILL for 3 sec)... ---"));
  calibrateGyroOffset(3000);
  Serial.print(F("Gyro Offset: ")); Serial.println(gyroZOffset);

  // 1. Turn Left (+90 Deg)
  Serial.println(F("\n[1/3] Turning LEFT +90.0 Degrees..."));
  executeTurn(+90.0);
  delay(1500);

  // 2. Turn Right (-90 Deg)
  Serial.println(F("[2/3] Turning RIGHT -90.0 Degrees..."));
  executeTurn(-90.0);
  delay(1500);

  // 3. Turn 180 Deg U-Turn (-180 Deg)
  Serial.println(F("[3/3] Turning U-TURN -180.0 Degrees..."));
  executeTurn(-180.0);
  delay(1000);

  Serial.println(F(">>> Precision Turning Test Complete."));
}

void executeTurn(float targetAngle) {
  currentYawAngle = 0.0;
  lastGyroMicros = micros();
  float absTarget = abs(targetAngle);
  unsigned long lastPrint = 0;

  while (abs(currentYawAngle) < absTarget - 2.0) {
    updateYawAngle();

    if (targetAngle < 0) {
      // Right Turn: Left Motor Fwd, Right Motor Rev (Angle goes NEGATIVE '-')
      analogWrite(LEFT_FWD,  TURN_SPEED); analogWrite(LEFT_REV,  0);
      analogWrite(RIGHT_FWD, 0);          analogWrite(RIGHT_REV, TURN_SPEED);
    } else {
      // Left Turn: Left Motor Rev, Right Motor Fwd (Angle goes POSITIVE '+')
      analogWrite(LEFT_FWD,  0);          analogWrite(LEFT_REV,  TURN_SPEED);
      analogWrite(RIGHT_FWD, TURN_SPEED); analogWrite(RIGHT_REV, 0);
    }

    // Print live angle every 100ms
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.print(F("  Turning... Angle: "));
      Serial.print(currentYawAngle, 2);
      Serial.println(F(" deg"));
    }

    delay(2);
  }
  stopMotors();

  float error = abs(targetAngle - currentYawAngle);
  Serial.println(F("  ---------------------------------------"));
  Serial.print(F("  Target Angle : ")); Serial.print(targetAngle, 2); Serial.println(F(" deg"));
  Serial.print(F("  Final Reached: ")); Serial.print(currentYawAngle, 2); Serial.println(F(" deg"));
  Serial.print(F("  Turn Error   : ")); Serial.print(error, 2); Serial.println(F(" deg"));
  Serial.println(F("  ---------------------------------------"));
}

// =============================================================
//  HELPER FUNCTIONS
// =============================================================
void stopMotors() {
  analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, 0);
  digitalWrite(BLUE_LED, LOW);
}

void initToFSensors() {
  pinMode(XSHUT_LEFT, OUTPUT);  digitalWrite(XSHUT_LEFT, LOW);
  pinMode(XSHUT_FRONT, OUTPUT); digitalWrite(XSHUT_FRONT, LOW);
  pinMode(XSHUT_RIGHT, OUTPUT); digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  if (tofLeft.init()) { tofLeft.setAddress(0x30); tofLeft.setTimeout(200); tofLeft.startContinuous(50); tofL_OK = true; }

  digitalWrite(XSHUT_FRONT, HIGH); delay(10);
  if (tofFront.init()) { tofFront.setAddress(0x31); tofFront.setTimeout(200); tofFront.startContinuous(50); tofF_OK = true; }

  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  if (tofRight.init()) { tofRight.setAddress(0x32); tofRight.setTimeout(200); tofRight.startContinuous(50); tofR_OK = true; }
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // Wake up MPU
  if (Wire.endTransmission(true) == 0) {
    mpu_OK = true;
  }
}
