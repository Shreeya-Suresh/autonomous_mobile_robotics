#!/usr/bin/env python3
import serial
import time
import sys

def check_serial(port='/dev/ttyUSB0', baudrate=115200):
    print(f"Opening {port} at {baudrate} baud...")
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2) # Wait for reboot
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return

    print("Listening for data... (Press Ctrl+C to stop)")
    print("Expected format: D,left,right,yaw")
    
    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("D,"):
                    print(f"Received: {line}")
                    parts = line.split(',')
                    if len(parts) >= 4: # D, l, r, yaw -> 4 parts
                        l = parts[1]
                        r = parts[2]
                        yaw = parts[3]
                        print(f"Parsed -> EncL:{l} EncR:{r} Yaw:{yaw}")
                else:
                    print(f"Raw: {line}")
            
    except KeyboardInterrupt:
        print("\nStopping...")
        ser.close()

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    check_serial(port)
