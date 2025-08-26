import serial
import time
import threading


motor1 = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=0.1)
motor2 = serial.Serial("/dev/ttyUSB1", baudrate=115200, timeout=0.1)

stop_threads = False  

def set_motor_speed(speed_1,speed2):
    if speed_1 > 0:
        dir1 = 0x00
    else:
        speed_1 = 65535 - abs(speed_1)
        dir1 = 0xFF

    speed1 = max(0, min(65535, speed1))
    speed_low_1 = speed1 & 0xFF
    speed_high_1 = (speed2 >> 8) & 0xFF
    if speed_2 > 0:
        dir2 = 0x00
    else:
        speed_2 = 65535 - abs(speed_2)
        dir2 = 0xFF

    speed2 = max(0, min(65535, speed2))
    speed_low_2 = speed2 & 0xFF
    speed_high_2 = (speed2 >> 8) & 0xFF

    cmd1 = [0xE0, 0x01, 0x00, 0x00, dir1, dir1, speed_high_1, speed_low_1]
    cmd2 = [0xE0, 0x01, 0x00, 0x00, dir2, dir2, speed_high_2, speed_low_2]
    motor1.write(bytearray(cmd1))
    motor2.write(bytearray(cmd2))

def to_signed_16(val):
    return val - (1 << 16) if val & 0x8000 else val

def to_signed_32(val):
    return val - (1 << 32) if val & 0x80000000 else val

def parse_frame(data):
    if len(data) != 13 or data[0] != 0xEE:
        return None
    electrical_angle = (data[1] << 8) | data[2]
    fault_code = (data[3] << 8) | data[4]
    temperature = data[5]
    voltage = data[6]
    speed_raw = (data[7] << 8) | data[8]
    speed = to_signed_16(speed_raw)
    position_raw = (data[9] << 24) | (data[10] << 16) | (data[11] << 8) | data[12]
    position = to_signed_32(position_raw)
    return {
        "ElectricalAngle": electrical_angle,
        "FaultCode": fault_code,
        "Temperature": temperature,
        "Voltage": voltage,
        "Speed": speed,
        "Position": position
    }
def get_encoder_postion():
    request = [0xED, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    if motor1 and motor1.is_open:
        motor1.write(bytearray(request))
        response= motor1.read(6)
    if len(response) == 6 and response[0] == 0xED and response[1] == 0x08:
        encoder_bytes = response[2:]  
        encoder_position = (encoder_bytes[0] << 24) | \
                   (encoder_bytes[1] << 16) | \
                   (encoder_bytes[2] << 8)  | \
                   encoder_bytes[3]

        if encoder_position & 0x80000000:
            encoder_position -= 0x100000000                  
        print("Encoder Position:", encoder_position)
    else:
        pass
    


def read_frame():
    global stop_threads
    while not stop_threads:
        time.sleep(0.05)
        if motor1 and motor1.is_open:
            data = motor1.read(13)
        if len(data) == 13:
            print(data)
            values = parse_frame(list(data))
            if values:
                print(values)
            else:
                print(f"Invalid frame: {[f'{b:02X}' for b in data]}")
        get_encoder_postion()
speed=1000
i=0
try:
    print("Reading motor driver data...")
    data_thread = threading.Thread(target=read_frame, daemon=True)
    data_thread.start()
    while True:
        set_motor_speed(10)
        time.sleep(0.02)
    
        

except KeyboardInterrupt:
    print("\nStopping...")
    stop_threads = True
    data_thread.join()
    motor1.close()
    motor2.close()
    print("Stopped.")
