import serial

# Open serial port
ser = serial.Serial('/dev/rs232tousb', 115200, timeout=0.1)  # adjust port & baudrate

print("Listening on serial port...")

try:
    while True:
        if ser.in_waiting > 0:  # check if data is available
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print("Received:", line)
except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    ser.close()
