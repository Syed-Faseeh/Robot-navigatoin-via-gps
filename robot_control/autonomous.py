import time
from .serial_manager import get_rtk
from .gps_utils import get_yaw, get_current_location, get_velocity_data, calculate_bearing, haversine, yaw_error, normalize_angle
from .motor_control import send_motor_command, send_stop, ramp_speed, clamp_pwm
from .state import state
from .config import ALPHA, BASE_SPEED, LOOP_DT, Kp_1, Ki_1, Kd_1, Kp_2, Ki_2, Kd_2, YAW_TOLERANCE, DISTANCE_THRESHOLD

def auto_turn():
    """Align robot heading to target bearing; exits immediately if state.COMMAND == 0."""
    rtk = get_rtk()
    print("[AUTO] Starting auto_turn")
    while True:
        if state.COMMAND == 0:
            send_stop()
            print("[AUTO] COMMAND 0 received: aborting auto_turn")
            return

        # Read one RTK line (non-blocking wrapper)
        line = None
        if rtk and rtk.is_open:
            try:
                line = rtk.readline().decode('ascii', errors='ignore').strip()
            except Exception:
                line = None

        if line:
            fields = line.split(',')
            if fields[0] == "#UNIHEADINGA":
                y = get_yaw(fields)
                if y is not None:
                    state.current_yaw = y
                    state.last_known_yaw = y
            elif fields[0] == "$GNGGA":
                lat, lon, _, _ = get_current_location(fields)
                if lat is not None and lon is not None:
                    state.lat = lat
                    state.lon = lon
                    state.last_known_lat = lat
                    state.last_known_lon = lon
            elif fields[0] == "$GNVTG":
                v = get_velocity_data(fields)
                if v is not None:
                    state.velocity = v
                    state.last_known_velocity = v

        # Use last-known for calculations
        use_lat = state.last_known_lat
        use_lon = state.last_known_lon
        use_yaw = state.last_known_yaw

        if use_lat is None or use_lon is None or use_yaw is None:
            # Wait briefly until initial fix but don't block telemetry
            time.sleep(0.05)
            continue

        if use_yaw < 0:
            use_yaw = 360 + use_yaw

        target_bearing = calculate_bearing(use_lat, use_lon, state.TARGET_LAT, state.TARGET_LON)
        if target_bearing is None:
            time.sleep(0.05)
            continue

        err, turn_speed = yaw_error(target_bearing, use_yaw)
        if err is None:
            time.sleep(0.05)
            continue

        print(f"[AUTO TURN] target:{target_bearing:.2f} curr:{use_yaw:.2f} err:{err:.2f}")

        if abs(err) < YAW_TOLERANCE:
            send_stop()
            print("[AUTO TURN] Aligned within tolerance.")
            return

        left_cmd = -turn_speed
        right_cmd = turn_speed
        send_motor_command(left_cmd, right_cmd)
        time.sleep(0.05)

def auto_move():
    """Move toward target with continuous yaw PID correction. Exits on state.COMMAND==0."""
    rtk = get_rtk()
    prev_yaw = state.last_known_yaw if state.last_known_yaw is not None else (state.current_yaw if state.current_yaw is not None else 0.0)
    filtered_yaw = prev_yaw
    prev_error = 0.0
    error_sum = 0.0
    state.left_pwm = BASE_SPEED
    state.right_pwm = BASE_SPEED

    print("[AUTO] Starting auto_move")
    while True:
        if state.COMMAND == 0:
            send_stop()
            print("[AUTO] COMMAND 0 received: aborting auto_move")
            return

        start_time = time.time()
        try:
            # Read one RTK line
            line = None
            if rtk and rtk.is_open:
                try:
                    line = rtk.readline().decode('ascii', errors='ignore').strip()
                except Exception:
                    line = None

            if line:
                fields = line.split(',')
                if fields[0] == "#UNIHEADINGA":
                    y = get_yaw(fields)
                    if y is not None:
                        state.current_yaw = y
                        state.last_known_yaw = y
                elif fields[0] == "$GNGGA":
                    lat, lon, _, _ = get_current_location(fields)
                    if lat is not None and lon is not None:
                        state.lat = lat
                        state.lon = lon
                        state.last_known_lat = lat
                        state.last_known_lon = lon
                elif fields[0] == "$GNVTG":
                    v = get_velocity_data(fields)
                    if v is not None:
                        state.velocity = v
                        state.last_known_velocity = v

            # Fallback to last-known
            use_lat = state.last_known_lat
            use_lon = state.last_known_lon
            use_yaw = state.last_known_yaw
            use_vel = state.last_known_velocity

            if use_lat is None or use_lon is None or use_yaw is None:
                print("[AUTO MOVE] missing data, waiting for fix...")
                time.sleep(0.05)
                continue

            if use_yaw < 0:
                use_yaw = 360 + use_yaw

            target_bearing = calculate_bearing(use_lat, use_lon, state.TARGET_LAT, state.TARGET_LON)
            if target_bearing is None:
                time.sleep(0.05)
                continue

            distance = haversine(use_lat, use_lon, state.TARGET_LAT, state.TARGET_LON)
            print(f"[AUTO MOVE] Distance: {distance:.2f}m | Target: {state.TARGET_LAT},{state.TARGET_LON}")

            # low-pass filtered yaw
            filtered_yaw = ALPHA * use_yaw + (1 - ALPHA) * prev_yaw
            prev_yaw = filtered_yaw

            # PID for heading
            heading_error = normalize_angle(target_bearing - filtered_yaw)
            error_sum += heading_error * LOOP_DT
            d_error = (heading_error - prev_error) / LOOP_DT if LOOP_DT > 0 else 0.0
            prev_error = heading_error

            control_output_M1 = Kp_1 * heading_error + Ki_1 * error_sum + Kd_1 * d_error
            control_output_M2 = Kp_2 * heading_error + Ki_2 * error_sum + Kd_2 * d_error

            target_left = BASE_SPEED + control_output_M1
            target_right = BASE_SPEED - control_output_M2

            state.left_pwm = ramp_speed(state.left_pwm, clamp_pwm(target_left))
            state.right_pwm = ramp_speed(state.right_pwm, clamp_pwm(target_right))

            print(f"Yaw: {filtered_yaw:.2f} | Err: {heading_error:.2f} | PWM L={state.left_pwm} R={state.right_pwm} | Vel: {use_vel}")

            if distance < DISTANCE_THRESHOLD:
                send_stop()
                print("[AUTO MOVE] Reached target.")
                return

            # send motion command
            send_motor_command(state.left_pwm, state.right_pwm)
            print (state.left_pwm, state.right_pwm)

            # maintain loop rate
            elapsed = time.time() - start_time
            time.sleep(max(0, LOOP_DT - elapsed))
        except Exception as e:
            print("[AUTO MOVE] exception:", e)
            time.sleep(0.05)
            continue
