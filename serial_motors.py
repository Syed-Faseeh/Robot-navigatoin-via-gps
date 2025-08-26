import threading
import time
import math
import serial

orin_lock = threading.Lock()
# --- Control Constants ---
Kp_1 = 8.0
Ki_1 = 0.8
Kd_1 = 0.0

Kp_2 = 8.0
Ki_2 = 0.8
Kd_2 = 0.0

ALPHA = 0.3
SPEED_RAMP = 1

LOOP_DT = 0.1  # 10 Hz

# --- State Variables ---
prev_yaw = 0
filtered_yaw = 0
prev_error = 0
error_sum = 0
BASE_SPEED = 750
m1_speed = BASE_SPEED
m2_speed = BASE_SPEED


global TARGET_LAT 
global TARGET_LON
TARGET_LAT = 0
TARGET_LON = 0
global COMMAND
COMMAND = 0

YAW_TOLERANCE = 0.5
DISTANCE_THRESHOLD = 0.5
lat,lon = 0.0,0.0
current_yaw = 0.0
velocity = 0.0
initial_yaw = None
voltage = 0.0

def initialize_serial():
    try:
        stm = serial.Serial('/dev/ttyUSB3', 9600, timeout=1)
        rtk = serial.Serial('/dev/rtk', 230400, timeout=1)
        orin = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        # motor1 = serial.Serial("/dev/motor1", baudrate=115200, timeout=0.1)
        # motor2 = serial.Serial("/dev/motor2", baudrate=115200, timeout=0.1)
        # return rtk, stm, orin, motor1, motor2
        return rtk, stm, orin
    except serial.SerialException as e:
        print(f"[ERROR] Serial init failed: {e}")
        return None
rtk, stm, orin= initialize_serial()
# rtk, stm, orin, motor1, motor2 = initialize_serial()
print("serial initiated")


def to_signed_16(val):
    return val - (1 << 16) if val & 0x8000 else val

def to_signed_32(val):
    return val - (1 << 32) if val & 0x80000000 else val

def motor_driver_1(data):
    electrical_angle = (data[1] << 8) | data[2]
    fault_code = (data[3] << 8) | data[4]
    temperature = data[5]
    voltage = data[6]
    speed_raw = (data[7] << 8) | data[8]
    speed = to_signed_16(speed_raw)
    position_raw = (data[9] << 24) | (data[10] << 16) | (data[11] << 8) | data[12]
    position = to_signed_32(position_raw)
    return voltage
    # return {
    #     "ElectricalAngle": electrical_angle,
    #     "FaultCode": fault_code,
    #     "Temperature": temperature,
    #     "Voltage": voltage,
    #     "Speed": speed,
    #     "Position": position
    # }

def get_yaw(sentence):
    try:
        yaw = float(sentence[12])
        print("YAW : ",yaw)
        yaw = yaw - 270
        return yaw
    except Exception as e:
        print(f"[ERROR] Yaw parse error: {e}")
        pass

def yaw_error(target, current):
    try:
        # print(target, current)
        error = target - current
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        if error < 0 :
            turn_speed = -250  - abs(0.6 * error)
        else :
            turn_speed = +250 + abs(0.6 * error)
        return error,turn_speed
    except:
        pass

def get_velocity_data(data):
    try:
        velocity = data[-3]
        return velocity
    except Exception as e:
        print(e)
        pass

def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius of Earth in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def calculate_bearing(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    bearing = math.atan2(x, y)
    return (math.degrees(bearing) + 360) % 360

def gngga_to_decimal(nmea_lat, nmea_lat_dir, nmea_lon, nmea_lon_dir):
    try:
        if(len(nmea_lat)>0):
            lat_degrees = int(nmea_lat[:2])
            lat_minutes = float(nmea_lat[2:])
            latitude = lat_degrees + (lat_minutes / 60)
            if nmea_lat_dir == 'S':
                latitude = -latitude
        else:
            print("RTK weak")   
            latitude=0.0
 
        if(len(nmea_lon) >0):
            lon_degrees = int(nmea_lon[:3])
            lon_minutes = float(nmea_lon[3:])
            longitude = lon_degrees + (lon_minutes / 60)
            if nmea_lon_dir == 'W':
                longitude = -longitude
        else:
            print("RTK weak")
            longitude=0.0

        return latitude, longitude
    except Exception as e:
        print(e)
        pass

def get_current_location(data):
    try:
        nmea_lat=data[2]
        nmea_lat_dir=data[3]
        nmea_lon=data[4]
        nmea_lon_dir=data[5]
        satellites = data[7]
        hdop = data[8]
        latitude,longitude=gngga_to_decimal(nmea_lat, nmea_lat_dir, nmea_lon, nmea_lon_dir)
        location=f"{latitude}, {longitude}"
        return latitude , longitude, satellites, hdop
    except Exception as e:
        print(e)
        pass

# --- Clamp PWM ---
def clamp_pwm(value, min_value=-1000, max_value=1000):
    return max(min_value, min(max_value, int(value)))

# --- Smooth Ramp ---
def ramp_speed(current, target):
    global SPEED_RAMP
    if current < target:
        return min(current + SPEED_RAMP, target)
    elif current > target: 
        return max(current - SPEED_RAMP, target)
    return current

def send_command(m1_val, m2_val):
    m1_val = clamp_pwm(m1_val)
    m2_val = clamp_pwm(m2_val)
    packet = f"M1:{m1_val},M2:{m2_val}\n"  # M2 inverted
    print("sent : |||||||" ,packet)
    if stm and stm.is_open:
        stm.write(packet.encode())

# def set_motor_speed(speed_1,speed2):
#     if speed_1 > 0:
#         dir1 = 0x00
#     else:
#         speed_1 = 65535 - abs(speed_1)
#         dir1 = 0xFF

#     speed1 = max(0, min(65535, speed1))
#     speed_low_1 = speed1 & 0xFF
#     speed_high_1 = (speed2 >> 8) & 0xFF
#     if speed_2 > 0:
#         dir2 = 0x00
#     else:
#         speed_2 = 65535 - abs(speed_2)
#         dir2 = 0xFF

#     speed2 = max(0, min(65535, speed2))
#     speed_low_2 = speed2 & 0xFF
#     speed_high_2 = (speed2 >> 8) & 0xFF

#     cmd1 = [0xE0, 0x01, 0x00, 0x00, dir1, dir1, speed_high_1, speed_low_1]
#     cmd2 = [0xE0, 0x01, 0x00, 0x00, dir2, dir2, speed_high_2, speed_low_2]
#     motor1.write(bytearray(cmd1))
#     motor2.write(bytearray(cmd2))

# --- Wait for Start ---

def compute_checksum(data: str) -> int:
    checksum = 0
    for char in data:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def data_thread():
    while True:
        try:
            time.sleep(1)
            payload = f"{lat:.11f},{lon:.11f},{current_yaw},{velocity},{voltage}"
            packet = f"<{payload}>\n"
            # print(packet)
            # print("to_orin : ", payload)
            orin.write(packet.encode())
        except serial.SerialException as se:
            print(f"[ERROR] Serial issue: {se}")
            time.sleep(1)  # Backoff before retrying
        except Exception as e:
            print(f"[ERROR] data_thread exception: {e}")
            time.sleep(0.1)

def gps_data():
    global current_yaw,lat,lon,velocity
    if rtk and rtk.is_open:
        line = rtk.readline().decode('ascii', errors='ignore').strip()
        data = line.split(',')
        if data:
            if data[0] == "#UNIHEADINGA":
                current_yaw = get_yaw(data)
            if data[0] == "$GNGGA":
                lat, lon , satellites, hdop= get_current_location(data)
            if data[0] == "$GNVTG":
                velocity = get_velocity_data(data)
        else:
            pass

        return current_yaw,lat,lon,velocity

def receiving_thread():
    global TARGET_LON, TARGET_LAT, COMMAND, m1_speed, m2_speed
    last_received_time = time.time()

    while True:
        rec_string = ""
        if orin and orin.is_open:
            while orin.in_waiting > 0:
                char = orin.readline().decode('ascii', errors='ignore').strip()
                rec_string += char
                if not rec_string:
                    continue  # skip empty lines
                
                # print(f"[Received] : {rec_string}")
                rec_packet = rec_string.split(',')
                data_str = ",".join(rec_packet[0:-1])
                check = rec_packet[-1]

                if  rec_packet[0] == "HB":
                    print(f"#################[RX] {rec_packet[0]}")
                    last_received_time = time.time()  # update heartbeat time
                if rec_packet[0] == "JS":
                    COMMAND = 2
                    m1_speed = float(rec_packet[1])
                    m2_speed = float(rec_packet[2])
                if rec_packet[0] == "AUTO":
                    try:
                        if check == compute_checksum(data_str):
                            TARGET_LAT = float(rec_packet[1])
                            TARGET_LON = float(rec_packet[2])
                            COMMAND = int(rec_packet[3])
                            print("[OK] checksum & parsed:", TARGET_LAT, TARGET_LON, COMMAND)
                            orin.write("ACK\n".encode())
                        else:
                            print("[WARN] checksum not okay")
                            orin.write("ACK--Failed\n".encode())
                    except ValueError as ve:
                        print(f"[WARN] Parse error: {ve}")
                # print(time.time()-last_received_time)
        if time.time() - last_received_time > 3:  
            if COMMAND != 0:  # only print once
                print("[WARN] CHECK CONNECTION")
                # COMMAND = 0 
                send_command(0,0) 
        time.sleep(0.01)
def auto_turn():
    global current_yaw, lat, lon, velocity
    while True:
        if rtk and rtk.is_open:
            try:
                # current_yaw,lat,lon,velocity = gps_data()
                line = rtk.readline().decode('ascii', errors='ignore').strip()
                data = line.split(',')
                if data:
                    if data[0] == "#UNIHEADINGA":
                        current_yaw = get_yaw(data)
                    if data[0] == "$GNGGA":
                        lat, lon , satellites, hdop= get_current_location(data)
                    if data[0] == "$GNVTG":
                        velocity = get_velocity_data(data)
                    else:
                        continue
                if lat is  None or lon is  None  or current_yaw is  None:
                    continue
                distance = haversine(lat, lon, TARGET_LAT, TARGET_LON)
                if COMMAND == 2:
                    print("Auto_turn: COMMAND = 2")
                    time.sleep(0.1)
                    break
                if current_yaw < 0:
                    current_yaw = 360 + current_yaw
                if COMMAND == 0 :
                    print("Auto_turn: COMMAND = 0")
                    time.sleep(0.05)
                    continue
                elif COMMAND == 1:
                    if distance < DISTANCE_THRESHOLD:
                        send_command(0, 0)
                        # set_motor_speed(0,0)
                        print("[INFO] Reached target.")
                        # COMMAND=0
                        time.sleep(0.05)
                        break
                    print("Auto_turn: COMMAND = 1")
                    target_bearing = calculate_bearing(lat, lon, TARGET_LAT, TARGET_LON)
                    error, turn_speed = yaw_error(target_bearing, current_yaw)
                    if abs(error) < YAW_TOLERANCE:
                        # set_motor_speed(0,0)
                        send_command(0, 0)
                        print(f"[INFO] Heading aligned. Moving to lat : {TARGET_LAT}, lon : {TARGET_LON}.")
                        break

                    send_command(-turn_speed, -turn_speed)
                    # set_motor_speed(turn_speed,turn_speed)
                    time.sleep(0.05)
                else:
                    time.sleep(0.1)
                    break
            except Exception as e:
                continue

def auto_move():
    global lon,lat,current_yaw,velocity
    BASE_SPEED = 400
    m1_speed = BASE_SPEED
    m2_speed = BASE_SPEED
    prev_yaw=current_yaw
    error_sum = 10
    SPEED_RAMP = 1
    prev_error = 0
    LOOP_DT = 0.1
    ALPHA = 0.3
    Kp_1 = 8.0
    Ki_1 = 1.0
    Kd_1 = 0.0
    Kp_2 = 8.0
    Ki_2 = 1.0
    Kd_2 = 0.0
    while True:
        if rtk and rtk.is_open:
            try:
                print("auto move ", COMMAND)
                start_time = time.time()
                # current_yaw,lat,lon,velocity = gps_data()
                line = rtk.readline().decode('ascii', errors='ignore').strip()
                data = line.split(',')
                if data:
                    if data[0] == "#UNIHEADINGA":
                        current_yaw = get_yaw(data)
                    if data[0] == "$GNGGA":
                        lat, lon , satellites, hdop= get_current_location(data)
                    if data[0] == "$GNVTG":
                        velocity = get_velocity_data(data)
                    else:
                        continue
                if lat is  None or lon is  None  or current_yaw is  None:
                    continue
                distance = haversine(lat, lon, TARGET_LAT, TARGET_LON)
                if current_yaw < 0:
                    current_yaw = 360 + current_yaw
                if COMMAND == 0 :
                    print("Auto_move: COMMAND = 0")
                    time.sleep(0.1)
                    break
                elif COMMAND == 1:
                    if distance < DISTANCE_THRESHOLD:
                        send_command(0, 0)
                        # set_motor_speed(0,0)
                        print("[INFO] Reached target.")
                        # COMMAND=0
                        time.sleep(0.05)
                        break
                    print("Auto_move: COMMAND = 1")
                    target_bearing = calculate_bearing(lat, lon, TARGET_LAT, TARGET_LON)
                    # print("Targets: ",TARGET_LAT,TARGET_LON)
                    
                    # distance = 50
                    filtered_yaw = ALPHA * current_yaw + (1 - ALPHA) * prev_yaw
                    prev_yaw = filtered_yaw
                    # PID
                    heading_error = normalize_angle(target_bearing - filtered_yaw)
                    error_sum += (2*heading_error) * LOOP_DT
                    d_error = (heading_error - prev_error) / LOOP_DT
                    prev_error = heading_error
                    # print(f"Target Bearing: {target_bearing:.2f}°, Current_Yaw: {current_yaw}, Velocity: {velocity},  Error: ")

                    control_output_M1 = Kp_1 * heading_error + Ki_1 * error_sum + Kd_1 * d_error
                    control_output_M2 = Kp_2 * heading_error + Ki_2 * error_sum + Kd_2 * d_error

                    # PWM
                    target_left = BASE_SPEED + control_output_M1
                    target_right = BASE_SPEED - control_output_M2

                    m1_speed = ramp_speed(m1_speed, clamp_pwm(target_left))
                    m2_speed = ramp_speed(m2_speed, clamp_pwm(target_right))

                    print(f"Target Bearing: {target_bearing:.2f}°, Yaw: {filtered_yaw:.2f}° |     Error: {heading_error:.2f}° | Error_Sum: {Ki_1*error_sum:.2f}° | PWM: L={m1_speed} R={m2_speed} | Velocity : {velocity}, Distance : {distance}")
                    print("command sent")
                    send_command(-m1_speed, m2_speed)
                    # set_motor_speed(m1_speed,-m2_speed)
                    
                    elapsed = time.time() - start_time
                    time.sleep(max(0, LOOP_DT - elapsed))
                else:
                    time.sleep(0.1)
                    break
            except Exception as e:
                continue  

def joystick():
    global lon,lat,current_yaw,velocity,m1_speed,m2_speed,COMMAND
    while True:
        try:
            if COMMAND == 2 :
                # current_yaw,lat,lon,velocity = gps_data()
                line = rtk.readline().decode('ascii', errors='ignore').strip()
                data = line.split(',')
                print (" JOYSTICK MODE" )
                send_command(-m1_speed,m2_speed)
                print(m1_speed,m2_speed)
                if data:
                    if data[0] == "#UNIHEADINGA":
                        current_yaw = get_yaw(data)
                    if data[0] == "$GNGGA":
                        lat, lon , satellites, hdop= get_current_location(data)
                    if data[0] == "$GNVTG":
                        velocity = get_velocity_data(data)
                    else:
                        continue
                print("inside joystick")
                if lat is  None or lon is  None  or current_yaw is  None:
                    continue
                # print("JOYSTICK")
                # set_motor_speed(-m1_speed,m2_speed)
                time.sleep(0.05)
            else:
                break
        except Exception as e:
            print(e)
            time.sleep(0.05)
            continue

# --- Main Loop ---
def main_loop():
    while True:
        joystick()
        auto_turn()
        auto_move()
        time.sleep(0.05)


if __name__ == "__main__":
    try:
        receiving = threading.Thread(target = receiving_thread)
        main_thread = threading.Thread(target = main_loop)
        data = threading.Thread(target = data_thread)
        # t1 = threading.Thread(target=serial_reader)
        # t2 = threading.Thread(target=watchdog)
        main_thread.start()
        receiving.start()
        data.start()
        # t1.start()
        # t2.start()
        print("All threads started")

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected. Stopping safely...")
        # set_motor_speed(0,0)
        send_command(0,0)
        main_thread.join()
        receiving.join()
        data.join()
        # t1.join()
        # t2.join()
        if rtk and rtk.is_open:
            rtk.close()
        if stm and stm.is_open:
            stm.close()
        if orin and orin.is_open:
            orin.close()
        print("[INFO] Serial closed. Exiting.")


