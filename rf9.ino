/*
  ===================================================================
  RESCUE MAZE - FULL AUTONOMOUS SOLVER
  ===================================================================
  Strategy: Wall-following (Straight first; if blocked, compare Left vs
            Right ToF distance and turn toward more open side; Back as last resort)
  Tile detection (TCS3200):
    BLUE  -> stop, blink LED for 5 seconds, then continue forward
    BLACK -> stop immediately, sound buzzer, treat as a dead end (turn 180),
             then keep navigating as usual -- since this maze is a single
             path, turning around here naturally retraces it back out
  Return-to-start: Dead-reckoning path log (heading + duration per straight
                    segment), replayed in reverse at heading+180.
                    Turning is now TIME-BASED (no gyro) -- the robot turns
                    for a calibrated duration per 90 degrees instead of
                    reading an MPU6050. "currentYaw" is a purely COMMANDED
                    heading (what we told the robot to turn to), not a
                    measured one, so it will drift over many turns just
                    like any open-loop dead reckoning.

  *** YOU MUST CALIBRATE THESE BEFORE RUNNING ON THE REAL ARENA ***
    - WALL_THRESH_MM      : distance (mm) below which a wall is "there"
    - TCS_BLUE_*, TCS_BLACK_* : raw TCS3200 pulse counts for your lighting
    - FORWARD_SPEED / TURN_SPEED : PWM values that move your bot reliably
    - TURN_90_DURATION_MS : how long (ms) it takes your bot to turn 90
                            degrees at TURN_SPEED -- MEASURE THIS ON THE
                            REAL ROBOT, it will not be correct by default.

  HOW "REACHED END POINT" WORKS:
    The rulebook doesn't define an automatic end-marker, so this sketch
    triggers the return sequence when you send the character 'E' over
    Serial (e.g., type E + Enter in Serial Monitor). If your arena DOES
    have a specific end marker (color/tile), tell me and I'll wire that
    in as an automatic trigger instead of the manual Serial command.

  Libraries required: VL53L0X by Pololu
  ===================================================================
*/

#include <Wire.h>
#include <VL53L0X.h>

// ---------------- PIN DEFINITIONS ----------------
#define XSHUT_LEFT   2
#define XSHUT_FRONT  3
#define XSHUT_RIGHT  4

#define TCS_S0  5
#define TCS_S1  6
#define TCS_S2  7
#define TCS_S3  8
#define TCS_OUT 9

#define ENA 10
#define ENB 11
#define IN1 12
#define IN2 13
#define IN3 A0
#define IN4 A1

#define BUZZER A2
#define LED    A3

#define ADDR_LEFT  0x30
#define ADDR_FRONT 0x31
#define ADDR_RIGHT 0x32

// ---------------- TUNABLE CONSTANTS ----------------
const int   WALL_THRESH_MM      = 90;   // front wall threshold -- calibrate
const int   SIDE_THRESH_MM      = 150;   // left/right open threshold (18 cm)
const int   FORWARD_SPEED       = 255;   // PWM 0-255
const int   TURN_SPEED          = 255;   // PWM 0-255
const unsigned long TURN_90_DURATION_MS  = 480; // ms to turn 90 deg on WHITE at TURN_SPEED
const unsigned long TURN_90_GREY_MS      = 480; // ms to turn 90 deg on GREY tile
const unsigned long TURN_180_DURATION_MS = 900; // ms to turn 180 deg at TURN_SPEED -- CALIBRATE THIS
const unsigned long BLACK_REVERSE_MS     = 1;   // ms to back up on a black tile (tiny nudge)
const int   MAX_PATH_SEGMENTS   = 30;    // path log capacity (plenty now RAM is freed up)

// TCS3200 raw thresholds -- derived from measured tile readings:
//   Grey floor  : R~30  G~11  B~35
//   White floor : R~32  G~11  B~30
//   Blue tile   : R~230 G~41  B~77
//   Black tile  : R~249 G~82  B~249
// pulseIn() here measures pulse WIDTH, so a BIGGER number = LESS light
// reflected (darker). Black is darkest overall -> highest raw numbers.
const int TCS_BLACK_B_THRESH = 150; // blue channel above this  -> black tile
const int TCS_BLUE_R_THRESH  = 100; // red channel above this (once black is ruled out) -> blue tile
const int TCS_GREY_B_THRESH  = 33;  // blue channel above this (once blue/black ruled out) -> grey (grey B~35 vs white B~30)

// ---------------- OBJECTS ----------------
VL53L0X tofLeft;
VL53L0X tofFront;
VL53L0X tofRight;

// ---------------- STATE ----------------
enum RobotState { NAVIGATING, RETURNING, STOPPED };
RobotState state = NAVIGATING;

enum TileColor { TILE_NONE, TILE_BLUE, TILE_BLACK, TILE_GREY };

const unsigned long BLUE_PAUSE_MS      = 2700; // how long to stop + blink on a blue tile
const unsigned long BLUE_BLINK_MS      = 250;  // LED on/off interval during the blue pause
const unsigned long BLUE_COOLDOWN_MS   = 2500; // ignore blue re-detection for this long after handling one
const unsigned long BLACK_BUZZ_MS      = 1000; // how long the buzzer sounds on a black hazard
unsigned long blueTileCooldownUntil = 0;
bool onGreySurface = false;  // tracks if robot is currently on grey (affects turn duration)

float currentYaw = 0.0;          // absolute heading, degrees -- COMMANDED value only (no sensor feedback)
unsigned long segmentStartTime = 0;

struct PathSegment {
  float heading;
  unsigned long duration;
};
PathSegment path[MAX_PATH_SEGMENTS];
int pathCount = 0;
int pathReturnIndex = -1;
int returnPhase = 0; // 0 = need to turn to segment heading, 1 = driving segment

// ===================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("========================================"));
  Serial.println(F(" RESCUE MAZE AUTONOMOUS SOLVER - BOOTING"));
  Serial.println(F("========================================"));

  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(TCS_S0, OUTPUT); pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT); pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  digitalWrite(TCS_S0, HIGH);  // 20% frequency scaling
  digitalWrite(TCS_S1, LOW);

  Wire.begin();
  setupToF();

  Serial.println(F("\n[READY] Robot will begin navigating in 3 seconds."));
  Serial.println(F("[READY] Send 'E' over Serial when the robot reaches the end point."));
  delay(3000);

  segmentStartTime = millis();
  Serial.println(F("\n[STATE] NAVIGATING\n"));
}

// ===================================================================
void loop() {
  handleSerialCommands();

  switch (state) {
    case NAVIGATING: runNavigation(); break;
    case RETURNING:  runReturn();     break;
    case STOPPED:    motorStop();     break;
  }
}

// ===================================================================
// SERIAL COMMANDS
// ===================================================================
void handleSerialCommands() {
  if (Serial.available()) {
    char c = Serial.read();
    if ((c == 'E' || c == 'e') && state == NAVIGATING) {
      Serial.println(F("\n[EVENT] End point signal received!"));
      motorStop();
      triggerReturnToStart();
    }
  }
}

// Closes out the in-progress segment and switches into RETURNING.
// Shared by the manual 'E' command and the automatic black-tile abort.
void triggerReturnToStart() {
  unsigned long dur = millis() - segmentStartTime;
  if (dur > 50) addPathSegment(currentYaw, dur);
  Serial.print(F("[PATH] Total segments recorded: "));
  Serial.println(pathCount);
  Serial.println(F("[STATE] RETURNING\n"));
  state = RETURNING;
  pathReturnIndex = pathCount - 1;
  returnPhase = 0;
}

// ===================================================================
// NAVIGATION (left-hand wall following, straight-first + L/R comparison)
// ===================================================================
void runNavigation() {
  int left  = readDistance(tofLeft);
  int front = readDistance(tofFront);
  int right = readDistance(tofRight);

  TileColor tile = checkTileColor();
  onGreySurface = (tile == TILE_GREY);  // affects turn duration: 580ms on grey, 500ms on white

  // BLACK is treated as a dead end: stop immediately, buzz, turn 180, and
  // keep navigating normally. Checked before any drive/turn command so we
  // never push further onto it.
  if (tile == TILE_BLACK) {
    handleBlackTileDeadEnd();
    return;
  }

  // BLUE is a pause-and-continue checkpoint, with a cooldown so the same
  // physical tile doesn't retrigger the pause every loop while we're still
  // sitting on top of it.
  if (tile == TILE_BLUE && millis() > blueTileCooldownUntil) {
    handleBlueTilePause();
    blueTileCooldownUntil = millis() + BLUE_COOLDOWN_MS;
    return;
  }

  bool leftOpen  = (left  < 0) || (left  > SIDE_THRESH_MM);
  bool frontOpen = (front < 0) || (front > WALL_THRESH_MM);
  bool rightOpen = (right < 0) || (right > SIDE_THRESH_MM);

  Serial.print(F("[NAV] L:")); Serial.print(left);
  Serial.print(F(" F:")); Serial.print(front);
  Serial.print(F(" R:")); Serial.print(right);
  Serial.print(F(" | Yaw:")); Serial.println(currentYaw);

  if (frontOpen) {
    // S - Straight first
    motorForward(FORWARD_SPEED);
  } else if (leftOpen && rightOpen) {
    // Both sides open -- compare actual distances and turn toward
    // whichever side has more room. Out-of-range (-1) = "wide open".
    int leftSpace  = (left  < 0) ? 32767 : left;
    int rightSpace = (right < 0) ? 32767 : right;
    if (leftSpace >= rightSpace) {
      Serial.println(F("[NAV] Straight blocked, both open -> LEFT has more space"));
      closeSegmentAndTurn(90);
    } else {
      Serial.println(F("[NAV] Straight blocked, both open -> RIGHT has more space"));
      closeSegmentAndTurn(-90);
    }
  } else if (leftOpen) {
    Serial.println(F("[NAV] Straight blocked, only Left open -> turning LEFT"));
    closeSegmentAndTurn(90);
  } else if (rightOpen) {
    Serial.println(F("[NAV] Straight blocked, only Right open -> turning RIGHT"));
    closeSegmentAndTurn(-90);
  } else {
    Serial.println(F("[NAV] Dead end -> turning BACK (180)"));
    closeSegmentAndTurn(180);
  }
}

void closeSegmentAndTurn(float deltaDeg) {
  unsigned long dur = millis() - segmentStartTime;
  if (dur > 50) addPathSegment(currentYaw, dur);
  motorStop();
  delay(100);
  turnByDegrees(deltaDeg);
  segmentStartTime = millis();
}

void addPathSegment(float heading, unsigned long duration) {
  if (pathCount < MAX_PATH_SEGMENTS) {
    path[pathCount].heading = heading;
    path[pathCount].duration = duration;
    pathCount++;
    Serial.print(F("[PATH] Logged segment #")); Serial.print(pathCount);
    Serial.print(F(" heading=")); Serial.print(heading);
    Serial.print(F(" dur=")); Serial.println(duration);
  } else {
    Serial.println(F("[PATH] WARNING: path log full, oldest data will be lost!"));
  }
}

// ===================================================================
// RETURN TO START (replay path reversed, each segment driven at +180 deg)
// ===================================================================
void runReturn() {
  if (pathReturnIndex < 0) {
    Serial.println(F("[RETURN] Complete! Robot should be back at start."));
    motorStop();
    state = STOPPED;
    return;
  }

  PathSegment seg = path[pathReturnIndex];
  float targetHeading = normalizeAngle(seg.heading + 180.0);

  if (returnPhase == 0) {
    Serial.print(F("[RETURN] Segment ")); Serial.print(pathReturnIndex + 1);
    Serial.print(F("/")); Serial.print(pathCount);
    Serial.print(F(" -> turning to heading ")); Serial.println(targetHeading);
    turnToAbsoluteHeading(targetHeading);
    returnPhase = 1;
    segmentStartTime = millis();
  } else {
    if (millis() - segmentStartTime < seg.duration) {
      motorForward(FORWARD_SPEED);
    } else {
      motorStop();
      Serial.println(F("[RETURN] Segment complete."));
      pathReturnIndex--;
      returnPhase = 0;
      delay(150);
    }
  }
}

// ===================================================================
// TOF SENSOR SETUP + READ
// ===================================================================
void setupToF() {
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  tofLeft.setTimeout(200);
  if (!tofLeft.init()) Serial.println(F("[FAIL] Left ToF not detected"));
  else {
    tofLeft.setAddress(ADDR_LEFT);
    Serial.println(F("[OK] Left ToF ready"));
  }

  digitalWrite(XSHUT_FRONT, HIGH); delay(10);
  tofFront.setTimeout(200);
  if (!tofFront.init()) Serial.println(F("[FAIL] Front ToF not detected"));
  else {
    tofFront.setAddress(ADDR_FRONT);
    Serial.println(F("[OK] Front ToF ready"));
  }

  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  tofRight.setTimeout(200);
  if (!tofRight.init()) Serial.println(F("[FAIL] Right ToF not detected"));
  else {
    tofRight.setAddress(ADDR_RIGHT);
    Serial.println(F("[OK] Right ToF ready"));
  }
}

int readDistance(VL53L0X &sensor) {
  int d = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) return -1;
  if (d >= 8190) return -1; // library's "out of range" sentinel value
  return d;
}

// ===================================================================
// HEADING TRACKING (open-loop / time-based, no gyro)
// ===================================================================
// currentYaw is COMMANDED, not measured: it only changes when we tell
// the robot to turn, by exactly the amount we told it to turn. There is
// no feedback, so real-world drift (motor mismatch, slipping wheels,
// surface friction) will NOT be corrected -- if turns drift off in
// testing, adjust TURN_90_DURATION_MS and/or add a per-wheel trim.

float normalizeAngle(float deg) {
  while (deg > 180) deg -= 360;
  while (deg < -180) deg += 360;
  return deg;
}

// Rotate by a relative amount (e.g. +90 = left, -90 = right, 180 = back)
void turnByDegrees(float deltaDeg) {
  Serial.print(F("[TURN] Turning by: ")); Serial.print(deltaDeg); Serial.println(F(" deg"));

  float absDelta = fabs(deltaDeg);
  // Pick turn duration based on surface: grey = 580ms, white = 500ms per 90 deg
  unsigned long turn90 = onGreySurface ? TURN_90_GREY_MS : TURN_90_DURATION_MS;
  unsigned long duration = (absDelta > 135.0) ? TURN_180_DURATION_MS : turn90;

  unsigned long turnStart = millis();
  if (deltaDeg > 0) {
    while (millis() - turnStart < duration) motorTurnLeft(TURN_SPEED);
  } else if (deltaDeg < 0) {
    while (millis() - turnStart < duration) motorTurnRight(TURN_SPEED);
  }
  motorStop();

  currentYaw = normalizeAngle(currentYaw + deltaDeg);
  Serial.print(F("[TURN] New commanded heading: ")); Serial.println(currentYaw);
  delay(150); // let the bot physically settle before next move
}

// Rotate to a fixed absolute heading (computed from the commanded currentYaw)
void turnToAbsoluteHeading(float targetDeg) {
  float delta = normalizeAngle(targetDeg - currentYaw);
  turnByDegrees(delta);
}

// ===================================================================
// TCS3200 COLOR DETECTION
// ===================================================================
int readColorFreq(bool s2, bool s3) {
  digitalWrite(TCS_S2, s2 ? HIGH : LOW);
  digitalWrite(TCS_S3, s3 ? HIGH : LOW);
  return pulseIn(TCS_OUT, LOW, 30000); // 30ms timeout, keeps loop responsive
}

TileColor checkTileColor() {
  int red   = readColorFreq(LOW, LOW);
  int blue  = readColorFreq(LOW, HIGH);
  int green = readColorFreq(HIGH, LOW);

  // pulseIn returns 0 on timeout -- treat 0 as "no reading", not "very high frequency"
  bool validReading = (red != 0 && green != 0 && blue != 0);
  if (!validReading) return TILE_NONE;

  if (blue > TCS_BLACK_B_THRESH) return TILE_BLACK;
  if (red  > TCS_BLUE_R_THRESH)  return TILE_BLUE;
  if (blue >= TCS_GREY_B_THRESH) return TILE_GREY;   // grey B~35 vs white B~30
  return TILE_NONE;  // white floor
}

// ===================================================================
// TILE EVENT HANDLERS (both blocking, matching the rest of this sketch's
// open-loop timing style)
// ===================================================================

// BLUE checkpoint: stop, blink the LED for BLUE_PAUSE_MS, then resume
// driving forward. The pause time is added back onto segmentStartTime so
// it isn't counted as "distance driven" in the path log / return replay.
void handleBlueTilePause() {
  Serial.println(F("[TILE] BLUE tile detected -> stopping, blinking LED"));
  motorStop();

  unsigned long pauseStart = millis();
  unsigned long nextToggle = pauseStart;
  bool ledOn = false;

  while (millis() - pauseStart < BLUE_PAUSE_MS) {
    if (millis() >= nextToggle) {
      ledOn = !ledOn;
      digitalWrite(LED, ledOn ? HIGH : LOW);
      nextToggle = millis() + BLUE_BLINK_MS;
    }
  }
  digitalWrite(LED, LOW);

  unsigned long pauseDuration = millis() - pauseStart;
  segmentStartTime += pauseDuration; // exclude the pause from driven-distance timing

  Serial.println(F("[TILE] Blue pause complete -> resuming forward"));
  motorForward(FORWARD_SPEED);
}

// BLACK hazard: stop the instant it's detected (before anything else),
// buzz, back up briefly, then turn 180 and stay in NAVIGATING -- treated
// like hitting a physical dead end. Since this maze is a single path,
// turning around here retraces the same corridor back out.
void handleBlackTileDeadEnd() {
  // Log the segment driven so far BEFORE any delays below.
  unsigned long dur = millis() - segmentStartTime;

  // 1. Stop immediately
  Serial.println(F("[TILE] BLACK tile detected -> stopping immediately"));
  motorStop();

  if (dur > 50) addPathSegment(currentYaw, dur);

  // 2. Reverse for just 1 ms (tiny nudge off the tile), then stop
  motorBackward(FORWARD_SPEED);
  delay(BLACK_REVERSE_MS);
  motorStop();

  // 3. Buzz for 1000 ms
  tone(BUZZER, 1000);
  delay(BLACK_BUZZ_MS);
  noTone(BUZZER);

  // 4. Turn 180 and continue navigating
  Serial.println(F("[TILE] Treating black tile as dead end -> turning BACK (180)"));
  turnByDegrees(180);
  segmentStartTime = millis();
}

// ===================================================================
// MOTOR CONTROL
// ===================================================================
void motorStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void motorForward(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}

void motorBackward(int speed) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}

void motorTurnLeft(int speed) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}

void motorTurnRight(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}
