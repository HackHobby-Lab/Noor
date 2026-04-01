#!/usr/bin/env python3
"""Simple serial monitor with UTF-8 support"""

import serial
import sys

PORT = "COM8"
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT} at {BAUD} baud", file=sys.stderr)
    
    while True:
        if ser.in_waiting:
            try:
                data = ser.read(ser.in_waiting)
                # Try UTF-8, fallback to latin1
                text = data.decode('utf-8', errors='replace')
                print(text, end='', flush=True)
            except Exception as e:
                print(f"Error: {e}", file=sys.stderr)
                
except KeyboardInterrupt:
    print("\nMonitoring stopped", file=sys.stderr)
    sys.exit(0)
except Exception as e:
    print(f"Error: {e}", file=sys.stderr)
    sys.exit(1)
