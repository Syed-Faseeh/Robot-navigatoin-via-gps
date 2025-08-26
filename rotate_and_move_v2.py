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
left_pwm = BASE_SPEED
right_pwm = BASE_SPEED


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

def initialize_serial():
    try:
        stm = serial.Serial('/dev/uart_converter', 9600, timeout=1)
        rtk = serial.Serial('/dev/rtk', 230400, timeout=1)
        orin = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        return rtk, stm, orin
    except serial.SerialException as e:
        print(f"[ERROR] Serial init failed: {e}")
        return None
rtk, stm, orin = initialize_serial()
print("serial initiated")

def get_yaw(sentence):
    try:
        yaw = float(sentence[12])
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
            print("RTK not connected to Satellites")   
            latitude=0
 
        if(len(nmea_lon) >0):
            lon_degrees = int(nmea_lon[:3])
            lon_minutes = float(nmea_lon[3:])
            longitude = lon_degrees + (lon_minutes / 60)
            if nmea_lon_dir == 'W':
                longitude = -longitude
        else:
            print("RTK not connected to Satellites")
            longitude=0

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

# --- Send Command to Motors ---
def send_motor_command(m1_val, m2_val):
    m1_val = clamp_pwm(m1_val)
    m2_val = clamp_pwm(m2_val)
    packet = f"M1:{m1_val},M2:{-m2_val}\n"  # M2 inverted
    print(packet)
    if stm and stm.is_open:
        stm.write(packet.encode())


def send_command(m1_val, m2_val):
    m1_val = clamp_pwm(m1_val)
    m2_val = clamp_pwm(m2_val)
    packet = f"M1:{-m1_val},M2:{+m2_val}\n"  # M2 inverted
    print(packet)
    if stm and stm.is_open:
        stm.write(packet.encode())

# --- Wait for Start ---

def compute_checksum(data: str) -> int:
    checksum = 0
    for char in data:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def data_thread():
    while True:
        try:
            payload = f"{lat:.11f},{lon:.11f},{current_yaw},{velocity}"
            packet = f"<{payload}>\n"
            # print("to_orin : ", payload)
            orin.write(packet.encode())
            time.sleep(0.5)
        except Exception as e:
            print(f"[ERROR] data_thread exception: {e}")
            time.sleep(0.1)
            continue

def receiving_thread():
    global TARGET_LON, TARGET_LAT, COMMAND
    while True:
        rec_string = ""
        if orin and orin.is_open:
            while orin.in_waiting > 0:
                time.sleep(0.05)
                char = orin.readline().decode('ascii', errors='ignore').strip()
                rec_string += char
                if not rec_string:
                    continue  # skip empty lines
                
                print(f"[RX] {rec_string}")
                rec_packet = rec_string.split(',')
                print(rec_packet)

                # Ensure we have at least 4 parts: lat, lon, command, checksum
                if len(rec_packet) < 4:
                    print("[WARN] Incomplete packet, skipping")
                    continue

                data_str = ",".join(rec_packet[0:-1])
                check = rec_packet[-1]

                if check == compute_checksum(data_str):
                    try:
                        TARGET_LAT = float(rec_packet[0])
                        TARGET_LON = float(rec_packet[1])
                        COMMAND = int(rec_packet[2])
                        print("[OK] checksum & parsed:", TARGET_LAT, TARGET_LON, COMMAND)
                    except ValueError as ve:
                        print(f"[WARN] Parse error: {ve}")
                else:
                    print("[WARN] checksum not okay")
        time.sleep(0.05)

def autonomous_mode():
    global current_yaw, lat, lon, velocity
    while True:
        if rtk and rtk.is_open:
            try:
                
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
                if current_yaw < 0:
                    current_yaw = 360 + current_yaw
                if COMMAND == 0 :
                    print("Auto_turn: COMMAND = 0")
                    time.sleep(0.05)
                    continue
                if COMMAND == 1:
                    print("Auto_turn: COMMAND = 1")
                    target_bearing = calculate_bearing(lat, lon, TARGET_LAT, TARGET_LON)
                    # print("Targets: ",TARGET_LAT,TARGET_LON)
                    # print('Current',current_yaw)
                    # print('target heading',target_bearing)
                    # print(f"Satellites : {satellites}, HDOP : {hdop},  Target Bearing: {target_bearing:.2f}°, Current_Yaw: {current_yaw}, Location: {lat}, {lon}, Velocity: {velocity} ")
                    error, turn_speed = yaw_error(target_bearing, current_yaw)
                    if abs(error) < YAW_TOLERANCE:
                        send_motor_command(0, 0)
                        print(f"[INFO] Heading aligned. Moving to lat : {TARGET_LAT}, lon : {TARGET_LON}.")
                        break

                    send_motor_command(-turn_speed, turn_speed)
                    time.sleep(0.1)
            except Exception as e:
                continue

def auto_move():
    global lon,lat,current_yaw,velocity
    Kp_1 = 8.0
    Ki_1 = 1.0
    Kd_1 = 0.0
    ALPHA = 0.3
    SPEED_RAMP = 1
    Kp_2 = 8.0
    Ki_2 = 1.0
    Kd_2 = 0.0
    LOOP_DT = 0.1  # 10 Hz
    prev_yaw=current_yaw
    BASE_SPEED = 800
    left_pwm = BASE_SPEED
    right_pwm = BASE_SPEED
    prev_error = 0
    error_sum = 10
    while True:
        try:
            if rtk and rtk.is_open:
                start_time = time.time()
                line = rtk.readline().decode('ascii', errors='ignore').strip()
                data = line.split(',')
                if data[0] == "#UNIHEADINGA":
                    current_yaw = get_yaw(data)
                if data[0] == "$GNGGA":
                    lat, lon , satellites, hdop= get_current_location(data)
                if data[0] == "$GNVTG":
                    velocity = get_velocity_data(data)
                else:
                    continue
                if current_yaw < 0:
                    current_yaw = 360 + current_yaw
                if lat is  None or lon is  None  or current_yaw is  None:
                    continue
                if COMMAND == 0 :
                    print("Auto_move: COMMAND = 0")
                    time.sleep(0.05)
                    break
                if COMMAND == 1:
                    print("Auto_move: COMMAND = 1")
                    target_bearing = calculate_bearing(lat, lon, TARGET_LAT, TARGET_LON)
                    # print("Targets: ",TARGET_LAT,TARGET_LON)
                    
                    # distance = 50
                    distance = haversine(lat, lon, TARGET_LAT, TARGET_LON)
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

                    left_pwm = ramp_speed(left_pwm, clamp_pwm(target_left))
                    right_pwm = ramp_speed(right_pwm, clamp_pwm(target_right))

                    print(f"Target Bearing: {target_bearing:.2f}°, Yaw: {filtered_yaw:.2f}° |     Error: {heading_error:.2f}° | Error_Sum: {Ki_1*error_sum:.2f}° | PWM: L={left_pwm} R={right_pwm} | Velocity : {velocity}, Distance : {distance}")
                    print("command sent")
                    if distance < DISTANCE_THRESHOLD:
                        send_motor_command(0, 0)
                        print("[INFO] Reached target.")
                        # COMMAND=0
                        time.sleep(0.05)
                        break
                    send_command(left_pwm, right_pwm)
                    
                    elapsed = time.time() - start_time
                    time.sleep(max(0, LOOP_DT - elapsed))
        except Exception as e:
            continue  
        

# --- Main Loop ---
def main_loop():
    while True:
        print("inside while")
        autonomous_mode()
        auto_move()
        time.sleep(0.05)


if __name__ == "__main__":
    try:
        main_thread = threading.Thread(target = main_loop)
        main_thread.start()
        print("main thread started")
        receiving_thread = threading.Thread(target = receiving_thread)
        receiving_thread.start()
        data_thread = threading.Thread(target = data_thread)
        data_thread.start()
        print("data thread started")
    except KeyboardInterrupt:
        main_thread.join()
        data_thread.join()
        print("\n[INFO] Ctrl+C detected. Stopping safely...")
        send_motor_command(0, 0)  # Stop motors
        if rtk and rtk.is_open:
            rtk.close()
        if stm and stm.is_open:
            stm.close()
        if orin and orin.is_open:
            orin.close()
        print("[INFO] Serial closed. Exiting.")
