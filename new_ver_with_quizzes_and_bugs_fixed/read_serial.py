import sys
import socket
import time

# Try to read from COM9 using Windows COM port
import msvcrt
import os

COM_PORT = "COM9"
print(f"Attempting to open {COM_PORT}...")

try:
    com_handle = os.open(COM_PORT, os.O_BINARY | os.O_RDONLY | os.O_NONBLOCK)
    print(f"Opened {COM_PORT}")
    
    # Read for 5 seconds
    end_time = time.time() + 5
    buffer = b""
    
    while time.time() < end_time:
        try:
            chunk = os.read(com_handle, 1024)
            if chunk:
                buffer += chunk
                sys.stdout.buffer.write(chunk)
                sys.stdout.flush()
        except:
            pass
        time.sleep(0.1)
    
    os.close(com_handle)
    print("\n\n=== CAPTURE COMPLETE ===")
    
except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)
