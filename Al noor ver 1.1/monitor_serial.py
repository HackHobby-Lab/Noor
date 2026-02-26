import subprocess
import time

print("Resetting device and reading serial output...")
time.sleep(1)

# Use esptool to reset device and then read
result = subprocess.run([
    "C:\\Espressif\\python_env\\idf5.5_py3.11_env\\Scripts\\python.exe", 
    "C:\\Espressif\\frameworks\\esp-idf-v5.5.2\\components\\esptool_py\\esptool\\esptool.py",
    "--chip", "esp32s3",
    "-p", "COM9",
    "-b", "115200",
    "--before", "default_reset",
    "read_flash",
    "0x0",
    "0x1"
], capture_output=True, text=True, timeout=10)

print(result.stdout)
print(result.stderr)

print("\nNow reading serial output for 10 seconds...")
time.sleep(2)

# Now try to read serial data
import sys
import threading

def read_serial():
    try:
        # Try using mode command to set COM9 parameters
        subprocess.run(["mode", "COM9:9600,N,8,1"], capture_output=True)
        
        # Now use PowerShell to read from serial port
        ps_cmd = """
$port = new-object System.IO.Ports.SerialPort COM9,115200,None,8,One
$port.open()
$port.WriteTimeout = 1000
$port.ReadTimeout = 1000
for($i=0; $i -lt 50; $i++) {
    try {
        $char = $port.ReadChar()
        [Console]::Write([char]$char)
    } catch {
        Start-Sleep -Milliseconds 100
    }
}
$port.close()
"""
        result = subprocess.run(["powershell", "-Command", ps_cmd], capture_output=True, text=True, timeout=15)
        print(result.stdout)
        if result.stderr:
            print("STDERR:", result.stderr)
    except Exception as e:
        print(f"Error reading serial: {e}")

read_serial()
