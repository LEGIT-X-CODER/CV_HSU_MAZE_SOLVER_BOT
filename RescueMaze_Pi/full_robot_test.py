#!/usr/bin/env python3
"""
============================================================
  MASTER ROBOT HARDWARE DIAGNOSTIC TEST SUITE
  Interactive CLI Menu via Pi SSH Terminal

  TESTS AVAILABLE:
    [1] Motor Test (Forward, Reverse, Left, Right, Stop)
    [2] ToF Sensor Test (Read Left, Front, Right distance cm)
    [3] Color Sensor Test (Read TCS3200 R,G,B & detect Black/Blue)
    [4] MPU6500 Gyro Test (Read Z-axis dps & live Yaw angle)
    [5] Turning Test (Precise +90° Left, -90° Right, 180° U-turn)
    [6] Buzzer & LED Test (Beep patterns & Blue LED flashes)
    [7] Servo Med-Kit Dispenser Test (Push 138° -> Pull 0°)
    [8] Victim Reaction Test (VICTIM H -> 2 kits, S -> 1 kit, U -> 0 kits)
    [9] TTP223 Touch Sensor Test (Read GPIO 17 state on Pi)
    [10] Auto Full System Diagnostic (Run all tests sequentially)

  USAGE:
    python3 full_robot_test.py
============================================================
"""

import sys
import time
import threading
import serial
import serial.tools.list_ports

BAUD = 115200

# ANSI Terminal Colors
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"


def find_port():
    candidates = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/serial0", "/dev/ttyAMA0"]
    for p in candidates:
        try:
            s = serial.Serial(p, BAUD, timeout=0.2)
            s.close()
            return p
        except Exception:
            pass
    for info in serial.tools.list_ports.comports():
        desc = (info.description or "").upper()
        if any(k in desc for k in ["ARDUINO", "CH340", "USB", "ACM"]):
            return info.device
    return None


class TestController:
    def __init__(self):
        self.port = find_port()
        self.ser = None
        self.running = True
        self.dist_cm = 999
        self.last_msg = ""
        self.lock = threading.Lock()

        if self.port:
            try:
                self.ser = serial.Serial(self.port, BAUD, timeout=0.2)
                time.sleep(2.0)
                threading.Thread(target=self._reader, daemon=True).start()
                print(f"{GREEN}[OK]{RESET} Connected to Arduino at {self.port}")
            except Exception as e:
                print(f"{RED}[ERROR]{RESET} Could not open serial port: {e}")
        else:
            print(f"{YELLOW}[WARN]{RESET} Arduino not connected (Serial tests disabled)")

    def _reader(self):
        while self.running and self.ser:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        with self.lock:
                            self.last_msg = line
                        if line.startswith("D:"):
                            try:
                                self.dist_cm = int(line[2:])
                            except ValueError:
                                pass
                            print(f"\r{CYAN}[ToF Live]{RESET} Front Distance = {self.dist_cm} cm          ", end="", flush=True)
                        else:
                            print(f"\n{GREEN}[Arduino Response]{RESET} {line}")
            except Exception:
                pass
            time.sleep(0.01)

    def send(self, cmd: str):
        if self.ser:
            try:
                self.ser.write((cmd + "\r\n").encode())
                self.ser.flush()
                print(f"\n{YELLOW}[Pi → Uno]{RESET} Sent: {BOLD}{cmd}{RESET}")
            except Exception as e:
                print(f"{RED}[Serial Error]{RESET} {e}")
        else:
            print(f"{RED}[Serial Error]{RESET} Arduino not connected")


def print_menu():
    print(f"\n{BOLD}{'='*60}{RESET}")
    print(f"{BOLD}   🤖 RESCUE MAZE BOT - MASTER HARDWARE DIAGNOSTIC SUITE{RESET}")
    print(f"{BOLD}{'='*60}{RESET}")
    print(f"  {GREEN}[1]{RESET}  Motor Test (Fwd, Rev, Left, Right, Stop)")
    print(f"  {GREEN}[2]{RESET}  ToF Distance Sensors Test (Left, Front, Right)")
    print(f"  {GREEN}[3]{RESET}  Color Sensor Test (TCS3200 R,G,B & Floor Detect)")
    print(f"  {GREEN}[4]{RESET}  MPU6500 Gyro Test (Read Z-Rate & Yaw Angle)")
    print(f"  {GREEN}[5]{RESET}  Gyro Turning Test (+90° Left, -90° Right, 180° U-Turn)")
    print(f"  {GREEN}[6]{RESET}  Buzzer & Blue LED Test")
    print(f"  {GREEN}[7]{RESET}  Servo Med-Kit Dispenser Test (138° Push -> 0° Pull)")
    print(f"  {GREEN}[8]{RESET}  Victim Reaction Test (H = 2 kits, S = 1 kit, U = 0 kits)")
    print(f"  {GREEN}[9]{RESET}  TTP223 Touch Sensor Test (GPIO 17 on Pi)")
    print(f"  {GREEN}[10]{RESET} Auto Run All Hardware Diagnostics")
    print(f"  {RED}[Q]{RESET}  Exit")
    print(f"{'─'*60}")


def test_touch_sensor():
    print(f"\n{CYAN}[9/10] Testing TTP223 Touch Sensor (GPIO 17)...{RESET}")
    try:
        import RPi.GPIO as GPIO
        TOUCH_PIN = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TOUCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        print("  --> Touch the TTP223 sensor now! (Testing for 5 seconds...)")
        end = time.time() + 5.0
        detected = False
        while time.time() < end:
            if GPIO.input(TOUCH_PIN) == GPIO.HIGH:
                print(f"  {GREEN}--> TOUCH DETECTED! (GPIO 17 HIGH){RESET}")
                detected = True
                time.sleep(0.3)
            time.sleep(0.05)
        if not detected:
            print(f"  {YELLOW}--> No touch detected during 5s window.{RESET}")
        GPIO.cleanup()
    except Exception as e:
        print(f"  {RED}--> GPIO Touch test error: {e}{RESET}")


def main():
    ctrl = TestController()

    while True:
        print_menu()
        choice = input(f"{BOLD}Select Option (1-10 or Q): {RESET}").strip().lower()

        if choice == 'q':
            print(f"\n{YELLOW}[EXIT]{RESET} Exiting test suite...")
            break

        elif choice == '1':
            print(f"\n{CYAN}[1] MOTOR TEST{RESET}")
            print("a: Forward 1s  |  b: Reverse 1s  |  c: Spin Left 1s  |  d: Spin Right 1s  |  s: Stop")
            sub = input("Select motor sub-test (a/b/c/d/s): ").strip().lower()
            cmds = {'a': 'TEST_MOTOR_FWD', 'b': 'TEST_MOTOR_REV', 'c': 'TEST_MOTOR_LEFT', 'd': 'TEST_MOTOR_RIGHT', 's': 'TEST_MOTOR_STOP'}
            if sub in cmds:
                ctrl.send(cmds[sub])
            else:
                print(f"{RED}Invalid sub-option{RESET}")

        elif choice == '2':
            print(f"\n{CYAN}[2] TOF DISTANCE SENSORS TEST{RESET}")
            ctrl.send("TEST_TOF")
            time.sleep(3.0)

        elif choice == '3':
            print(f"\n{CYAN}[3] TCS3200 COLOR SENSOR TEST{RESET}")
            ctrl.send("TEST_COLOR")
            time.sleep(3.0)

        elif choice == '4':
            print(f"\n{CYAN}[4] MPU GYRO LIVE REALTIME ANGLE TEST (10 Seconds Stream){RESET}")
            print(f"{YELLOW}--> Rotate the robot by hand to see live integrated Yaw angle update!{RESET}")
            ctrl.send("STREAM_MPU")
            time.sleep(10.5)

        elif choice == '5':
            print(f"\n{CYAN}[5] GYRO TURNING TEST{RESET}")
            print("l: Left (+90°)  |  r: Right (-90°)  |  u: U-Turn (180°)")
            sub = input("Select turn (l/r/u): ").strip().lower()
            turn_map = {'l': 'TURN_LEFT', 'r': 'TURN_RIGHT', 'u': 'TURN_UTURN'}
            if sub in turn_map:
                ctrl.send(turn_map[sub])
            else:
                print(f"{RED}Invalid turn option{RESET}")
            time.sleep(3.0)

        elif choice == '6':
            print(f"\n{CYAN}[6] BUZZER & LED TEST{RESET}")
            ctrl.send("TEST_BUZZER_LED")
            time.sleep(2.0)

        elif choice == '7':
            print(f"\n{CYAN}[7] SERVO MED-KIT DISPENSER TEST{RESET}")
            ctrl.send("TEST_SERVO")
            time.sleep(3.0)

        elif choice == '8':
            print(f"\n{CYAN}[8] VICTIM REACTION TEST{RESET}")
            sub = input("Select Victim Type (h = Harmed / s = Stable / u = Unharmed): ").strip().upper()
            if sub in ['H', 'S', 'U']:
                ctrl.send(f"VICTIM:{sub}")
            else:
                print(f"{RED}Invalid victim type{RESET}")
            time.sleep(3.0)

        elif choice == '9':
            test_touch_sensor()

        elif choice == '10':
            print(f"\n{BOLD}{CYAN}=== STARTING AUTOMATED FULL SYSTEM HARDWARE DIAGNOSTIC ==={RESET}")
            ctrl.send("TEST_BUZZER_LED"); time.sleep(2.0)
            ctrl.send("TEST_TOF"); time.sleep(2.0)
            ctrl.send("TEST_COLOR"); time.sleep(2.0)
            ctrl.send("TEST_MPU"); time.sleep(2.0)
            ctrl.send("TEST_SERVO"); time.sleep(3.0)
            test_touch_sensor()
            print(f"\n{GREEN}{BOLD}[DIAGNOSTIC COMPLETE]{RESET}")

        else:
            print(f"{RED}Invalid option! Select 1-10 or Q{RESET}")

        time.sleep(1.0)

    ctrl.running = False
    if ctrl.ser:
        ctrl.ser.close()


if __name__ == "__main__":
    main()
