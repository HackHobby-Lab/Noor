#!/usr/bin/env python3
import serial
import time
import sys

try:
    print("Opening COM8...", file=sys.stderr)
    s = serial.Serial('COM8', 115200, timeout=2)
    time.sleep(1)
    
    print("Reading from serial buffer...", file=sys.stderr)
    output = s.read(8192)
    
    if output:
        text = output.decode('utf-8', errors='replace')
        print("\n=== SERIAL OUTPUT ===")
        print(text)
        print("=== END OUTPUT ===\n")
    else:
        print("No data received from serial port", file=sys.stderr)
    
    s.close()
    print("Port closed successfully", file=sys.stderr)
    
except Exception as e:
    print(f"Error: {e}", file=sys.stderr)
    sys.exit(1)
