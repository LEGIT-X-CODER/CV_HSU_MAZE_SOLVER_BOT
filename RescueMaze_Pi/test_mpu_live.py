#!/usr/bin/env python3
"""
============================================================
  MPU-6050 REALTIME ANGLE VIEWER (Pi SSH Terminal)
  
  Sends STREAM_MPU command to Arduino and displays live 
  Yaw angle and Z-rate in real-time.
============================================================
"""

import sys
import time
import serial
import serial.tools.list_ports

BAUD = 115200

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

def main():
    port = find_port()
    if not port:
        print(f"{BOLD}\033[91m[ERROR] Could not find Arduino serial port! Check connections.{RESET}")
        sys.exit(1)

    print(f"{BOLD}{CYAN}=========================================================={RESET}")
    print(f"{BOLD}{CYAN}  MPU-6050 REALTIME ANGLE STREAM (Pi SSH Terminal){RESET}")
    print(f"{BOLD}{CYAN}=========================================================={RESET}")
    print(f"{GREEN}[OK]{RESET} Connecting to Arduino at {port}...")

    ser = serial.Serial(port, BAUD, timeout=0.5)
    time.sleep(2.0)

    print(f"{YELLOW}--> Starting MPU Live Stream... Press Ctrl+C to exit.{RESET}\n")

    try:
        while True:
            # Request MPU stream
            ser.write(b"STREAM_MPU\r\n")
            ser.flush()
            
            start = time.time()
            while time.time() - start < 14.5:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("MPU_LIVE:") or "Yaw=" in line:
                        print(f"\r{BOLD}{GREEN}[LIVE ANGLE]{RESET} {line}                          ", end="", flush=True)
                    elif line:
                        print(f"\n{CYAN}[Arduino]{RESET} {line}")
                time.sleep(0.05)
    except KeyboardInterrupt:
        print(f"\n\n{YELLOW}[EXIT]{RESET} Exiting MPU Live Stream.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
