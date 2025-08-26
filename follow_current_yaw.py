import time
import math
import serial

# --- Control Constants ---
Kp_1 = 30.0
Ki_1 = 10.0
Kd_1 = 0.0

Kp_2 = 30.0
Ki_2 = 10.0
Kd_2 = 0.0

ALPHA = 0.3
SPEED_RAMP = 5
BASE_SPEED = 400
LOOP_DT = 0.1  # 10 Hz

# --- State Variables ---
prev_yaw = 0
filtered_yaw = 0
prev_error = 0
error_sum = 0
left_pwm = BASE_SPEED
right_pwm = BASE_SPEED
  # Jetson Nano (Linux)
# --- Serial Initialization ---
def initialize_serial():
    try:
        ser2 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        ser = serial.Serial('/dev/ttyUSB1', 230400, timeout=1)
        return ser, ser2
    except serial.SerialException as e:
        print(f"[ERROR] Serial init failed: {e}")
        return None

ser = initialize_serial()


def get_velocity_data(data):
    try:
        if data[0] == "$GNVTG":
            velocity = data[-3]
            print(f"Velocity = {velocity}")

    except Exception as e:
        print(e)
        pass


# --- Yaw Parsing ---
def get_yaw(sentence):
    try:
        if sentence[0] == "#UNIHEADINGA":
            yaw = float(sentence[12])
            print("yaw:", yaw)
            return yaw
    except Exception as e:
        print(f"[ERROR] Yaw parse error: {e}")
    return None

# --- Normalize Error to [-180, 180] ---
def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

# --- Clamp PWM ---
def clamp_pwm(value, min_value=-1000, max_value=1000):
    return max(min_value, min(max_value, int(value)))

# --- Smooth Ramp ---
def ramp_speed(current, target):
    if current < target:
        return min(current + SPEED_RAMP, target)
    elif current > target: 
        return max(current - SPEED_RAMP, target)
    return current

# --- Send Command to Motors ---
def send_command(m1_val, m2_val):
    m1_val = clamp_pwm(m1_val)
    m2_val = clamp_pwm(m2_val)
    packet = f"M1:{m1_val},M2:{-m2_val}\n"  # M2 inverted
    print(packet)
    if ser2 and ser2.is_open:
        ser2.write(packet.encode())

# --- Wait for Start ---
initial_yaw = None
ser, ser2 = initialize_serial()
print("serial initiated")
while True:
        while initial_yaw is None:
            if ser and ser.is_open:
                try:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    data = line.split(',')
                    initial_yaw = get_yaw(data)
                except:
                    continue
        prev_yaw = initial_yaw
        print(f"[STARTED] Initial heading locked at {initial_yaw:.2f}°")
        break

# --- Main Loop ---
while True:
    start_time = time.time()

    # Read Yaw
    current_yaw = None
    if ser and ser.is_open:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            data = line.split(',')
            current_yaw = get_yaw(data)
            velocity = get_velocity_data(data)
        except:
            continue

    if current_yaw is None:
        continue

    # Filter Yaw
    filtered_yaw = ALPHA * current_yaw + (1 - ALPHA) * prev_yaw
    prev_yaw = filtered_yaw

    # PID
    heading_error = normalize_angle(initial_yaw - filtered_yaw)
    error_sum += heading_error * LOOP_DT
    d_error = (heading_error - prev_error) / LOOP_DT
    prev_error = heading_error

    control_output_M1 = Kp_1 * heading_error + Ki_1 * error_sum + Kd_1 * d_error
    control_output_M2 = Kp_2 * heading_error + Ki_2 * error_sum + Kd_2 * d_error

    # PWM
    target_left = BASE_SPEED - control_output_M1
    target_right = BASE_SPEED + control_output_M2

    left_pwm = ramp_speed(left_pwm, clamp_pwm(target_left))
    right_pwm = ramp_speed(right_pwm, clamp_pwm(target_right))

    # Send to STM
    send_command(left_pwm, right_pwm)

    # Debug
    print(f"Yaw: {filtered_yaw:.2f}° | Velocity: {velocity:.2f}° |  Error: {heading_error:.2f}° | Error_Sum: {Ki_1*error_sum:.2f}° | PWM: L={left_pwm} R={right_pwm}")

    # Maintain Loop Rate
    elapsed = time.time() - start_time
    time.sleep(max(0, LOOP_DT - elapsed))
