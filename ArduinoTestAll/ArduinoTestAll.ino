// ============================================================
//  ARDUINO ALL-IN-ONE INTERACTIVE TEST SKETCH
//  
//  SERIAL COMMANDS (Open Serial Monitor at 115200 baud):
//    '1' or 'm' - Test Motors (Forward, Reverse, Spins + Indications)
//    '2' or 't' - Test ToF Distance Sensors (Left, Front, Right)
//    '3' or 'c' - Test TCS3200 Color Sensor (R, G, B & Tile Detection)
//    '4' or 'i' - Test MPU6500 IMU Angle (Live Yaw Angle in DEGREES)
//    '5' or 'z' - Test Turning (Left +90°, Right -90°, -180° U-Turn)
//    '6' or 'b' - Test Buzzer & Blue LED
//    '7' or 's' - Test Med-Kit Dispenser Servo (Pin 8 Sweep & Drop)
//    '8' or 'v' - Test Victim Reaction (H = 2 kits, S = 1 kit, U = 0 kits)
//    '10' or 'a'- Run All Hardware Diagnostics Sequentially
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
int TURN_SPEED    = 180;   // Turning motor speed for 90/180 turns (0-255)

const int MPU_ADDR = 0x68;

// ---- SENSOR OBJECTS & GLOBALS -------------------------------
VL53L0X tofLeft, tofFront, tofRight;
bool tofL_OK = false, tofF_OK = false, tofR_OK = false;
bool mpu_OK = false;

float gyroZOffset = 0.0;
float currentYawAngle = 0.0;
unsigned long lastGyroMicros = 0;

// Function Prototypes
void testMotors();
void testToF();
void testColorSensor();
void testIMUAngle();
void testTurning();
void testBuzzerLED();
void testServo();
void testVictimReaction(char letter);
void testAll();
void printMenu();
void printMenuPrompt();
void stopMotors();
void servoWrite(int angle);
void soundIndicate(int beeps, bool solidLED);
int readDistance(VL53L0X &sensor, bool isOK);
void printDistVal(int cm);
int readColorPulse(bool s2, bool s3);
void calibrateGyroOffset(unsigned long durationMs);
int16_t readRawGyroZ();
void updateYawAngle();
void executeTurn(float targetDeg);


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
//  MAIN LOOP - Serial Command Parser (Matches 1 to 10 options!)
// =============================================================
void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    Serial.print(F("\n>>> EXECUTING COMMAND: '"));
    Serial.print(input);
    Serial.println(F("'"));

    String cmdUpper = input;
    cmdUpper.toUpperCase();

    // Check for Victim reaction command (e.g. VICTIM:H, VICTIM:S, VICTIM:U)
    if (cmdUpper.startsWith("VICTIM:")) {
      char letter = cmdUpper.charAt(7);
      testVictimReaction(letter);
      printMenuPrompt();
      return;
    }

    // Custom Servo Angle handling: e.g., 's0', 's90', 's180', 's138'
    if ((input.charAt(0) == 's' || input.charAt(0) == 'S') && input.length() > 1 && input != "s" && input != "S") {
      int targetAngle = input.substring(1).toInt();
      targetAngle = constrain(targetAngle, 0, 180);
      Serial.print(F(">>> Moving Servo (Pin 8) to Angle: "));
      Serial.print(targetAngle);
      Serial.println(F(" deg"));
      servoWrite(targetAngle);
      printMenuPrompt();
      return;
    }

    // Option Match (1-10 or single character commands)
    if (cmdUpper == "1" || cmdUpper == "M" || cmdUpper.startsWith("MOTOR") || cmdUpper.startsWith("TEST_MOTOR")) {
      testMotors();
    } else if (cmdUpper == "2" || cmdUpper == "T" || cmdUpper == "TOF" || cmdUpper.startsWith("TEST_TOF")) {
      testToF();
    } else if (cmdUpper == "3" || cmdUpper == "C" || cmdUpper == "COLOR" || cmdUpper.startsWith("TEST_COLOR")) {
      testColorSensor();
    } else if (cmdUpper == "4" || cmdUpper == "I" || cmdUpper == "IMU" || cmdUpper == "MPU" || cmdUpper.startsWith("STREAM") || cmdUpper.startsWith("TEST_MPU")) {
      testIMUAngle();
    } else if (cmdUpper == "5" || cmdUpper == "Z" || cmdUpper.startsWith("TURN") || cmdUpper.startsWith("TEST_TURN")) {
      testTurning();
    } else if (cmdUpper == "6" || cmdUpper == "B" || cmdUpper == "L" || cmdUpper.startsWith("BUZZER") || cmdUpper.startsWith("TEST_BUZZER")) {
      testBuzzerLED();
    } else if (cmdUpper == "7" || cmdUpper == "S" || cmdUpper.startsWith("SERVO") || cmdUpper.startsWith("TEST_SERVO")) {
      testServo();
    } else if (cmdUpper == "8" || cmdUpper == "V" || cmdUpper.startsWith("VICTIM")) {
      testVictimReaction('H'); // Default H test
    } else if (cmdUpper == "10" || cmdUpper == "A" || cmdUpper == "ALL" || cmdUpper.startsWith("TEST_ALL")) {
      testAll();
    } else if (cmdUpper == "?" || cmdUpper == "H" || cmdUpper == "HELP" || cmdUpper == "MENU") {
      printMenu();
    } else {
      Serial.println(F("Unknown command! Options: 1-10 or 'm','t','c','i','z','b','s','a'."));
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
  Serial.println(F("  [1/m] - Test Motors (Forward, Reverse, Spins + Indications)"));
  Serial.println(F("  [2/t] - Test ToF Distance Sensors (Left, Front, Right)"));
  Serial.println(F("  [3/c] - Test TCS3200 Color Sensor (R, G, B & Tile Type)"));
  Serial.println(F("  [4/i] - Test MPU6500 Live Yaw Angle (Shows actual Yaw in deg)"));
  Serial.println(F("  [5/z] - Test Turning (Left +90 deg, Right -90 deg, -180 deg)"));
  Serial.println(F("  [6/b] - Test Buzzer & Blue LED"));
  Serial.println(F("  [7/s] - Test Servo Dispenser (Pin 8 Sweep & Kit Drop)"));
  Serial.println(F("  [8/v] - Test Victim Reaction (H = 2 kits, S = 1 kit, U = 0 kits)"));
  Serial.println(F("  [10/a]- Run All Hardware Diagnostics Sequentially"));
  Serial.println(F("=================================================="));
  printMenuPrompt();
}

void printMenuPrompt() {
  Serial.print(F("\nEnter command (1-10 or m/t/c/i/z/b/s/v/a): "));
}

// =============================================================
//  COMMAND [2]: TEST ToF SENSORS
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
//  COMMAND [1]: TEST MOTORS WITH INDICATION
// =============================================================
void testMotors() {
  Serial.println(F("--- Motor Test with Audio/Visual Indications ---"));
  
  // 1. Left Motor Forward
  Serial.println(F("1/6 Left Motor FORWARD (1 beep, LED solid)"));
  soundIndicate(1, true);
  analogWrite(LEFT_FWD, FORWARD_SPEED); analogWrite(LEFT_REV, 0);
  delay(1000); stopMotors(); delay(400);

  // 2. Left Motor Reverse
  Serial.println(F("2/6 Left Motor REVERSE (1 beep, LED blink)"));
  soundIndicate(1, false);
  analogWrite(LEFT_FWD, 0); analogWrite(LEFT_REV, FORWARD_SPEED);
  delay(1000); stopMotors(); delay(400);

  // 3. Right Motor Forward
  Serial.println(F("3/6 Right Motor FORWARD (2 beeps, LED solid)"));
  soundIndicate(2, true);
  analogWrite(RIGHT_FWD, FORWARD_SPEED); analogWrite(RIGHT_REV, 0);
  delay(1000); stopMotors(); delay(400);

  // 4. Right Motor Reverse
  Serial.println(F("4/6 Right Motor REVERSE (2 beeps, LED blink)"));
  soundIndicate(2, false);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, FORWARD_SPEED);
  delay(1000); stopMotors(); delay(400);

  // 5. Both Forward
  Serial.println(F("5/6 Both Motors FORWARD (3 beeps, LED solid)"));
  soundIndicate(3, true);
  analogWrite(LEFT_FWD, FORWARD_SPEED);  analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, FORWARD_SPEED); analogWrite(RIGHT_REV, 0);
  delay(1000); stopMotors(); delay(400);

  // 6. Both Reverse
  Serial.println(F("6/6 Both Motors REVERSE (3 beeps, LED blink)"));
  soundIndicate(3, false);
  analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, FORWARD_SPEED);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, FORWARD_SPEED);
  delay(1000); stopMotors(); delay(400);

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
//  COMMAND [4]: TEST MPU6500 IMU ANGLE (Live Yaw Angle Tracking)
// =============================================================
void testIMUAngle() {
  if (!mpu_OK) {
    Serial.println(F("ERROR: MPU6500 not connected or failed initialization!"));
    return;
  }

  Serial.println(F("--- Calibrating Gyro Offset (KEEP ROBOT STILL for 2 sec)... ---"));
  calibrateGyroOffset(2000);
  Serial.print(F("Calibrated Gyro Offset: ")); Serial.println(gyroZOffset);

  Serial.println(F("\n--- Live Yaw Angle Tracking (10 Seconds) ---"));
  Serial.println(F("Rotate the bot by hand to see the ANGLE IN DEGREES update!"));

  currentYawAngle = 0.0;
  lastGyroMicros = micros();
  unsigned long testStart = millis();

  while (millis() - testStart < 10000) {
    updateYawAngle();
    
    // Print every 200ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
      lastPrint = millis();
      Serial.print(F("Current Yaw Angle: "));
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
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  Wire.read(); Wire.read(); Wire.read(); Wire.read();
  return ((int16_t)Wire.read() << 8) | Wire.read();
}

void updateYawAngle() {
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastGyroMicros) / 1000000.0;
  lastGyroMicros = nowMicros;

  int16_t rawZ = readRawGyroZ();
  float dps = (rawZ / 131.0) - (gyroZOffset / 131.0);
  currentYawAngle += dps * dt;
}

// =============================================================
//  COMMAND [6]: TEST BUZZER & BLUE LED
// =============================================================
void testBuzzerLED() {
  Serial.println(F("--- Testing Buzzer Tones & Blue LED (A0) ---"));
  tone(BUZZER, 1000, 150); digitalWrite(BLUE_LED, HIGH); delay(200); digitalWrite(BLUE_LED, LOW);
  tone(BUZZER, 1500, 150); digitalWrite(BLUE_LED, HIGH); delay(200); digitalWrite(BLUE_LED, LOW);
  tone(BUZZER, 2000, 150); digitalWrite(BLUE_LED, HIGH); delay(200); digitalWrite(BLUE_LED, LOW);
  tone(BUZZER, 2500, 300); digitalWrite(BLUE_LED, HIGH); delay(350); digitalWrite(BLUE_LED, LOW);
  Serial.println(F(">>> Buzzer & LED Test Complete."));
}

// =============================================================
//  COMMAND [3]: TEST TCS3200 COLOR SENSOR
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
  return (int)pulseIn(TCS_OUT, LOW, 6000);
}

// =============================================================
//  COMMAND [7]: TEST SERVO DISPENSER (PIN 8)
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

// =============================================================
//  COMMAND [8]: TEST VICTIM REACTION
// =============================================================
void testVictimReaction(char letter) {
  Serial.print(F("--- Testing Victim Reaction for Letter: ")); Serial.print(letter); Serial.println(F(" ---"));
  stopMotors();

  if (letter == 'H' || letter == 'h') {
    Serial.println(F("  --> Victim H: 2 Med-Kits Drop + 3 Beeps"));
    tone(BUZZER, 2000, 150); delay(200); tone(BUZZER, 2000, 150); delay(200); tone(BUZZER, 2000, 150); delay(200);
    servoWrite(138); delay(400); servoWrite(0); delay(400);
    servoWrite(138); delay(400); servoWrite(0); delay(400);
  } else if (letter == 'S' || letter == 's') {
    Serial.println(F("  --> Victim S: 1 Med-Kit Drop + 2 Beeps"));
    tone(BUZZER, 2000, 150); delay(200); tone(BUZZER, 2000, 150); delay(200);
    servoWrite(138); delay(400); servoWrite(0); delay(400);
  } else if (letter == 'U' || letter == 'u') {
    Serial.println(F("  --> Victim U: 0 Kits + 1 Beep (Unharmed)"));
    tone(BUZZER, 2000, 300); delay(350);
  }
  Serial.println(F(">>> Victim Reaction Test Complete."));
}

// =============================================================
//  COMMAND [5]: TEST PRECISE TURNS (MPU Closed-Loop)
// =============================================================
void testTurning() {
  if (!mpu_OK) {
    Serial.println(F("ERROR: MPU6500 required for turns!"));
    return;
  }

  Serial.println(F("--- Calibrating Gyro Offset (KEEP STILL for 2 sec)... ---"));
  calibrateGyroOffset(2000);

  // 1. Turn Left (+90 Deg)
  Serial.println(F("\n[1/3] Turning LEFT +90.0 Degrees..."));
  executeTurn(+90.0);
  delay(1000);

  // 2. Turn Right (-90 Deg)
  Serial.println(F("[2/3] Turning RIGHT -90.0 Degrees..."));
  executeTurn(-90.0);
  delay(1000);

  // 3. Turn U-Turn (-180 Deg)
  Serial.println(F("[3/3] Turning U-TURN -180.0 Degrees..."));
  executeTurn(-180.0);
  delay(1000);

  Serial.println(F(">>> Turning Test Complete."));
}

void executeTurn(float targetDeg) {
  stopMotors();
  delay(100);
  currentYawAngle = 0.0;
  lastGyroMicros = micros();

  unsigned long start = millis();
  unsigned long timeout = 900;
  int turnPwm = 165;
  float targetMag = abs(targetDeg) - 4.0;

  // Soft start ramp-up
  for (int s = 70; s <= turnPwm; s += 30) {
    if (targetDeg > 0) {
      analogWrite(LEFT_FWD, 0);        analogWrite(LEFT_REV, s);
      analogWrite(RIGHT_FWD, s);       analogWrite(RIGHT_REV, 0);
    } else {
      analogWrite(LEFT_FWD, s);        analogWrite(LEFT_REV, 0);
      analogWrite(RIGHT_FWD, 0);       analogWrite(RIGHT_REV, s);
    }
    delay(10);
  }

  while (millis() - start < timeout) {
    updateYawAngle();
    if (abs(currentYawAngle) >= targetMag) break;

    if (targetDeg > 0) {
      analogWrite(LEFT_FWD, 0);        analogWrite(LEFT_REV, turnPwm);
      analogWrite(RIGHT_FWD, turnPwm); analogWrite(RIGHT_REV, 0);
    } else {
      analogWrite(LEFT_FWD, turnPwm);  analogWrite(LEFT_REV, 0);
      analogWrite(RIGHT_FWD, 0);       analogWrite(RIGHT_REV, turnPwm);
    }
    delay(5);
  }
  stopMotors();
}

// =============================================================
//  COMMAND [10]: AUTOMATED ALL HARDWARE TESTS
// =============================================================
void testAll() {
  Serial.println(F("\n=================================================="));
  Serial.println(F("   STARTING AUTOMATED ALL HARDWARE DIAGNOSTICS   "));
  Serial.println(F("=================================================="));

  testBuzzerLED(); delay(500);
  testToF();       delay(500);
  testColorSensor(); delay(500);
  testIMUAngle();  delay(500);
  testServo();     delay(500);
  testTurning();   delay(500);

  Serial.println(F("\n=================================================="));
  Serial.println(F("   ALL HARDWARE DIAGNOSTICS COMPLETE!           "));
  Serial.println(F("==================================================\n"));
}

// =============================================================
//  HARDWARE INITS & HELPERS
// =============================================================
void initToFSensors() {
  pinMode(XSHUT_LEFT, OUTPUT); pinMode(XSHUT_FRONT, OUTPUT); pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW); digitalWrite(XSHUT_FRONT, LOW); digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);

  digitalWrite(XSHUT_LEFT, HIGH); delay(30);
  if (tofLeft.init()) { tofLeft.setAddress(0x30); tofLeft.setTimeout(100); tofLeft.startContinuous(30); tofL_OK = true; Serial.println(F("  Left  ToF OK (0x30)")); }
  else Serial.println(F("  Left  ToF FAIL!"));

  digitalWrite(XSHUT_FRONT, HIGH); delay(30);
  if (tofFront.init()) { tofFront.setAddress(0x31); tofFront.setTimeout(100); tofFront.startContinuous(30); tofF_OK = true; Serial.println(F("  Front ToF OK (0x31)")); }
  else Serial.println(F("  Front ToF FAIL!"));

  digitalWrite(XSHUT_RIGHT, HIGH); delay(30);
  if (tofRight.init()) { tofRight.setAddress(0x32); tofRight.setTimeout(100); tofRight.startContinuous(30); tofR_OK = true; Serial.println(F("  Right ToF OK (0x32)")); }
  else Serial.println(F("  Right ToF FAIL!"));
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  if (Wire.endTransmission(true) == 0) {
    mpu_OK = true;
    Serial.println(F("  MPU6500 IMU OK (0x68)"));
  } else {
    mpu_OK = false;
    Serial.println(F("  MPU6500 IMU FAIL!"));
  }
}

void stopMotors() {
  analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, 0);
}
