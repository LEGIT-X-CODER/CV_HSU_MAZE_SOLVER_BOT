// ============================================================
//  MOTOR TEST WITH LED + BUZZER INDICATION
//  
//  Each motor movement has a unique beep pattern + LED signal
//  so you can tell exactly which motor is running even with
//  your eyes closed!
//
//  PINS:
//    Left Motor:  D5 (fwd), D6 (rev)
//    Right Motor: D9 (fwd), D10 (rev)
//    Buzzer: D3    |    Blue LED: A0
//
//  No libraries needed. Just upload and open Serial at 115200.
// ============================================================

#define LEFT_FWD   5
#define LEFT_REV   6
#define RIGHT_FWD  9
#define RIGHT_REV  10
#define BUZZER     3
#define BLUE_LED   A0

#define SPEED      130   // Test speed (0-255)
#define RUN_TIME   1500  // ms each motor runs

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_FWD,  OUTPUT); pinMode(LEFT_REV,  OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT); pinMode(RIGHT_REV, OUTPUT);
  pinMode(BUZZER,    OUTPUT);
  pinMode(BLUE_LED,  OUTPUT);
  stopAll();

  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("   MOTOR TEST WITH INDICATION"));
  Serial.println(F("========================================"));
  Serial.println(F(""));
  Serial.println(F("Starting in 3 seconds..."));
  
  // Countdown with beeps
  for (int i = 3; i >= 1; i--) {
    Serial.print(i); Serial.println(F("..."));
    tone(BUZZER, 1000, 100);
    digitalWrite(BLUE_LED, HIGH); delay(200);
    digitalWrite(BLUE_LED, LOW);  delay(800);
  }

  Serial.println(F("GO!\n"));
  tone(BUZZER, 2000, 300); delay(400);

  // =============== TEST 1: LEFT MOTOR FORWARD ================
  // Indication: 1 beep + LED solid ON
  Serial.println(F("[1/8] LEFT Motor FORWARD"));
  Serial.println(F("      Signal: 1 beep + LED solid"));
  indicate(1, true);
  analogWrite(LEFT_FWD, SPEED); analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, 0);    analogWrite(RIGHT_REV, 0);
  delay(RUN_TIME);
  stopAll();
  delay(600);

  // =============== TEST 2: LEFT MOTOR REVERSE ================
  // Indication: 1 beep + LED blinking
  Serial.println(F("[2/8] LEFT Motor REVERSE"));
  Serial.println(F("      Signal: 1 beep + LED blinking"));
  indicate(1, false);
  analogWrite(LEFT_FWD, 0);     analogWrite(LEFT_REV, SPEED);
  analogWrite(RIGHT_FWD, 0);    analogWrite(RIGHT_REV, 0);
  blinkWhileRunning(RUN_TIME);
  stopAll();
  digitalWrite(BLUE_LED, LOW);
  delay(600);

  // =============== TEST 3: RIGHT MOTOR FORWARD ===============
  // Indication: 2 beeps + LED solid ON
  Serial.println(F("[3/8] RIGHT Motor FORWARD"));
  Serial.println(F("      Signal: 2 beeps + LED solid"));
  indicate(2, true);
  analogWrite(LEFT_FWD, 0);     analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, SPEED); analogWrite(RIGHT_REV, 0);
  delay(RUN_TIME);
  stopAll();
  delay(600);

  // =============== TEST 4: RIGHT MOTOR REVERSE ===============
  // Indication: 2 beeps + LED blinking
  Serial.println(F("[4/8] RIGHT Motor REVERSE"));
  Serial.println(F("      Signal: 2 beeps + LED blinking"));
  indicate(2, false);
  analogWrite(LEFT_FWD, 0);     analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, 0);    analogWrite(RIGHT_REV, SPEED);
  blinkWhileRunning(RUN_TIME);
  stopAll();
  digitalWrite(BLUE_LED, LOW);
  delay(600);

  // =============== TEST 5: BOTH FORWARD (STRAIGHT) ===========
  // Indication: 3 beeps + LED solid
  Serial.println(F("[5/8] BOTH Motors FORWARD (Straight)"));
  Serial.println(F("      Signal: 3 beeps + LED solid"));
  indicate(3, true);
  analogWrite(LEFT_FWD, SPEED); analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, SPEED); analogWrite(RIGHT_REV, 0);
  delay(RUN_TIME);
  stopAll();
  delay(600);

  // =============== TEST 6: BOTH REVERSE ======================
  // Indication: 3 beeps + LED blinking
  Serial.println(F("[6/8] BOTH Motors REVERSE"));
  Serial.println(F("      Signal: 3 beeps + LED blinking"));
  indicate(3, false);
  analogWrite(LEFT_FWD, 0);     analogWrite(LEFT_REV, SPEED);
  analogWrite(RIGHT_FWD, 0);    analogWrite(RIGHT_REV, SPEED);
  blinkWhileRunning(RUN_TIME);
  stopAll();
  digitalWrite(BLUE_LED, LOW);
  delay(600);

  // =============== TEST 7: SPIN LEFT (left rev + right fwd) ==
  // Indication: Long beep + rapid blink
  Serial.println(F("[7/8] SPIN LEFT (In-place Turn Left)"));
  Serial.println(F("      Signal: long beep + rapid blink"));
  tone(BUZZER, 1500, 500); delay(600);
  analogWrite(LEFT_FWD, 0);     analogWrite(LEFT_REV, SPEED);
  analogWrite(RIGHT_FWD, SPEED); analogWrite(RIGHT_REV, 0);
  rapidBlinkWhileRunning(RUN_TIME);
  stopAll();
  digitalWrite(BLUE_LED, LOW);
  delay(600);

  // =============== TEST 8: SPIN RIGHT (left fwd + right rev) =
  // Indication: 2 long beeps + rapid blink
  Serial.println(F("[8/8] SPIN RIGHT (In-place Turn Right)"));
  Serial.println(F("      Signal: 2 long beeps + rapid blink"));
  tone(BUZZER, 1500, 200); delay(300);
  tone(BUZZER, 1500, 200); delay(300);
  analogWrite(LEFT_FWD, SPEED); analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, 0);    analogWrite(RIGHT_REV, SPEED);
  rapidBlinkWhileRunning(RUN_TIME);
  stopAll();
  digitalWrite(BLUE_LED, LOW);
  delay(600);

  // =============== DONE ======================================
  Serial.println(F("\n========================================"));
  Serial.println(F("   ALL MOTOR TESTS COMPLETE!"));
  Serial.println(F("========================================"));
  Serial.println(F(""));
  Serial.println(F("CHECK LIST:"));
  Serial.println(F("  [1] Left  wheel spun forward?     Y/N"));
  Serial.println(F("  [2] Left  wheel spun reverse?     Y/N"));
  Serial.println(F("  [3] Right wheel spun forward?     Y/N"));
  Serial.println(F("  [4] Right wheel spun reverse?     Y/N"));
  Serial.println(F("  [5] Bot went STRAIGHT forward?    Y/N"));
  Serial.println(F("  [6] Bot went STRAIGHT backward?   Y/N"));
  Serial.println(F("  [7] Bot spun LEFT in place?       Y/N"));
  Serial.println(F("  [8] Bot spun RIGHT in place?      Y/N"));
  Serial.println(F(""));
  Serial.println(F("If any motor is reversed, swap its 2 wires"));
  Serial.println(F("on the L298N output terminal."));

  // Victory tune
  tone(BUZZER, 2000, 100); delay(120);
  tone(BUZZER, 2500, 100); delay(120);
  tone(BUZZER, 3000, 200); delay(250);

  // Blink 5 times = done
  for (int i = 0; i < 5; i++) {
    digitalWrite(BLUE_LED, HIGH); delay(100);
    digitalWrite(BLUE_LED, LOW);  delay(100);
  }
}


void loop() {
  // Nothing in loop - test runs once on power-up
  // Press Arduino RESET button to run again
}


// =============================================================
//  HELPERS
// =============================================================
void stopAll() {
  analogWrite(LEFT_FWD, 0);  analogWrite(LEFT_REV, 0);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_REV, 0);
}

// Beep N times, then turn LED on (solid) or leave off (for blink mode)
void indicate(int beeps, bool ledSolid) {
  for (int i = 0; i < beeps; i++) {
    tone(BUZZER, 2000, 100);
    digitalWrite(BLUE_LED, HIGH);
    delay(150);
    digitalWrite(BLUE_LED, LOW);
    delay(100);
  }
  if (ledSolid) digitalWrite(BLUE_LED, HIGH);
  delay(200);
}

// Blink LED at ~2Hz while motor runs (used for reverse direction)
void blinkWhileRunning(int durationMs) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)durationMs) {
    digitalWrite(BLUE_LED, HIGH); delay(250);
    digitalWrite(BLUE_LED, LOW);  delay(250);
  }
}

// Rapid blink at ~8Hz while motor runs (used for spin turns)
void rapidBlinkWhileRunning(int durationMs) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)durationMs) {
    digitalWrite(BLUE_LED, HIGH); delay(60);
    digitalWrite(BLUE_LED, LOW);  delay(60);
  }
}
