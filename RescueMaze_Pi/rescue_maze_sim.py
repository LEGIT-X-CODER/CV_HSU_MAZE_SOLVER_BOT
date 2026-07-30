#!/usr/bin/env python3
"""
Rescue Maze - Pi Serial Bridge
- Shows ALL raw data from Arduino live
- Type h / s / u + Enter → sends VICTIM:H/S/U to Arduino
- Type q → quit

Run ON PI via SSH:
  python3 rescue_maze_sim.py
"""

import sys
import time
import threading
import serial
import serial.tools.list_ports

BAUD = 115200

RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"


def find_port():
    # USB serial first, then GPIO UART ports
    candidates = [
        "/dev/ttyACM0", "/dev/ttyACM1",    # Arduino USB
        "/dev/ttyUSB0", "/dev/ttyUSB1",    # CH340 USB
        "/dev/serial0",                     # Pi GPIO UART (hardware)
        "/dev/ttyAMA0",                     # Pi GPIO UART (alternate)
        "/dev/ttyS0",                       # Pi mini-UART
    ]
    for p in candidates:
        try:
            s = serial.Serial(p, BAUD, timeout=0.1)
            s.close()
            print(f"[Found] {p}")
            return p
        except Exception:
            pass
    # Auto-detect via pyserial
    for info in serial.tools.list_ports.comports():
        desc = (info.description or "").upper()
        if any(k in desc for k in ["ARDUINO", "CH340", "USB SERIAL", "ACM", "UART"]):
            return info.device
    return None


def reader_thread(ser, running):
    """Print everything coming from Arduino."""
    while running[0]:
        try:
            if ser.in_waiting:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue

                if raw.startswith("D:"):
                    try:
                        d = int(raw[2:])
                        col = RED if d <= 10 else (YELLOW if d <= 20 else GREEN)
                        tag = " <<< VICTIM ZONE" if d <= 10 else ""
                        # Print on same line for distance (overwrite)
                        print(f"\r{col}[DIST]{RESET} {d} cm{tag}          ",
                              end="", flush=True)
                    except ValueError:
                        print(f"\r{CYAN}[Arduino]{RESET} {raw}          ")
                else:
                    # All other messages: print on new line clearly
                    print(f"\n{CYAN}[Arduino]{RESET} {raw}")
        except Exception:
            pass
        time.sleep(0.01)


def main():
    print(f"\n{BOLD}{'='*50}{RESET}")
    print(f"{BOLD}  RESCUE MAZE - PI SERIAL BRIDGE{RESET}")
    print(f"{BOLD}{'='*50}{RESET}\n")

    # Find Arduino
    port = find_port()
    if not port:
        print(f"{RED}[ERROR]{RESET} Arduino not found!")
        print(f"Check: ls /dev/ttyACM* /dev/ttyUSB*")
        sys.exit(1)

    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
    except Exception as e:
        print(f"{RED}[ERROR]{RESET} Cannot open {port}: {e}")
        sys.exit(1)

    print(f"{GREEN}[OK]{RESET} Arduino @ {port}  |  {BAUD} baud")
    time.sleep(2.0)  # wait for Arduino reset after serial connect

    # Start background reader
    running = [True]
    t = threading.Thread(target=reader_thread, args=(ser, running), daemon=True)
    t.start()

    # Instructions
    print(f"\n{BOLD}Manual SSH Commands:{RESET}")
    print(f"  {GREEN}start{RESET} or {GREEN}go{RESET} → Send START signal to Arduino (begins maze)")
    print(f"  {YELLOW}h{RESET}          → Send VICTIM:H (Harmed)")
    print(f"  {YELLOW}s{RESET}          → Send VICTIM:S (Stable)")
    print(f"  {YELLOW}u{RESET}          → Send VICTIM:U (Unharmed)")
    print(f"  {RED}q{RESET}          → Quit\n")
    print(f"{'─'*50}")

    # Input loop
    try:
        while True:
            cmd = input().strip().lower()

            if cmd == "q":
                break
            elif cmd in ("start", "go"):
                ser.write(b"START\r\n")
                ser.flush()
                print(f"\n{YELLOW}[Pi → Uno]{RESET} {BOLD}START{RESET}")
            elif cmd in ("h", "s", "u"):
                letter = cmd.upper()
                msg = f"VICTIM:{letter}\r\n"
                ser.write(msg.encode())
                ser.flush()
                print(f"\n{YELLOW}[Pi → Uno]{RESET} {BOLD}VICTIM:{letter}{RESET}")
            elif cmd == "":
                pass
            else:
                print(f"{RED}[?]{RESET} Use: start / h / s / u / q")

    except KeyboardInterrupt:
        pass

    print(f"\n\n{YELLOW}[EXIT]{RESET} Stopping...")
    running[0] = False
    ser.close()


if __name__ == "__main__":
    main()
