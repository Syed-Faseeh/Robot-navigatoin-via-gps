import threading
import time
import math
import serial
from dataclasses import dataclass, field

# =========================
# Config & Constants
# =========================
LOOP_DT = 0.1                 # 10 Hz
YAW_TOLERANCE = 0.5           # degrees
DISTANCE_THRESHOLD = 0.5      # meters
TELEMETRY_DT = 0.5            # seconds
BASE_SPEED = 800              # PWM base forward
MAX_PWM = 1000

# Gains (can be tuned)
Kp = 8.0
Ki = 1.0
Kd = 0.0
ALPHA = 0.3                   # yaw smoothing alpha
SPEED_RAMP = 1                # PWM ramp step per cycle
INTEGRAL_LIMIT = 200.0        # anti-windup clamp for integral term

# =========================
# Helper math
# =========================
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def clamp_pwm(v): return int(clamp(v, -MAX_PWM, MAX_PWM))

def normalize_angle(angle):
    """Normalize to (-180, 180]"""
    while angle > 180: angle -= 360
    while angle <= -180: angle += 360
    return angle

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2.0)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2.0)**2
    return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

def bearing_deg(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlambda)
    brg = math.degrees(math.atan2(x, y))
    return (brg + 360.0) % 360.0

def compute_checksum_hex(s: str) -> str:
    cs = 0
    for ch in s:
        cs ^= ord(ch)
    return f"{cs:02X}"

# =========================
# NMEA Parsing
# =========================
def parse_gngga(fields):
    # $GNGGA,time,lat,NS,lon,EW,fix,sats,hdop,alt,unit,geo,unit2,age,ref*CS
    # indexes:         2   3   4   5        7    8
    try:
        nmea_lat = fields[2]
        nmea_lat_dir = fields[3]
        nmea_lon = fields[4]
        nmea_lon_dir = fields[5]
        sats = fields[7]
        hdop = fields[8]

        if nmea_lat and nmea_lon:
            lat_deg = int(nmea_lat[:2])
            lat_min = float(nmea_lat[2:])
            lat = lat_deg + lat_min/60.0
            if nmea_lat_dir == 'S': lat = -lat

            lon_deg = int(nmea_lon[:3])
            lon_min = float(nmea_lon[3:])
            lon = lon_deg + lon_min/60.0
            if nmea_lon_dir == 'W': lon = -lon
        else:
            return None

        return lat, lon, sats, hdop
    except Exception:
        return None

def parse_uniheadinga(fields):
    # Example: #UNIHEADINGA,<...>,<yaw>,<...>
    # You used fields[12] for yaw then yaw-270. Keep that mapping.
    try:
        yaw = float(fields[12]) - 270.0
        # Normalize 0..360
        if yaw < 0: yaw += 360.0
        return yaw
    except Exception:
        return None

def parse_gnvtg(fields):
    # $GNVTG, ... , speed_kts, ... or vendor-specific. You used data[-3]
    try:
        vel = fields[-3]
        return float(vel) if vel else 0.0
    except Exception:
        return 0.0

# =========================
# Shared State
# =========================
@dataclass
class RobotState:
    lat: float = 0.0
    lon: float = 0.0
    yaw: float = 0.0
    velocity: float = 0.0

    target_lat: float = 0.0
    target_lon: float = 0.0
    command: int = 0   # 0 = idle, 1 = go-to

    # PID memory
    prev_yaw_filt: float = 0.0
    err_prev: float = 0.0
    err_sum: float = 0.0
    left_pwm: int = BASE_SPEED
    right_pwm: int = BASE_SPEED

# =========================
# Controller
# =========================
class RobotController:
    def __init__(self,
                 stm_port='/dev/uart_converter', stm_baud=9600,
                 rtk_port='/dev/rtk', rtk_baud=230400,
                 orin_port='/dev/ttyUSB1', orin_baud=115200):
        self.state = RobotState()
        self.state_lock = threading.Lock()

        self.stop_event = threading.Event()

        # Serial
        try:
            self.stm = serial.Serial(stm_port, stm_baud, timeout=1)
        except Exception as e:
            print(f"[ERROR] STM init failed: {e}")
            self.stm = None

        try:
            self.rtk = serial.Serial(rtk_port, rtk_baud, timeout=1)
        except Exception as e:
            print(f"[ERROR] RTK init failed: {e}")
            self.rtk = None

        try:
            self.orin = serial.Serial(orin_port, orin_baud, timeout=1)
        except Exception as e:
            print(f"[ERROR] ORIN init failed: {e}")
            self.orin = None

        # Write locks for serial ports
        self.stm_wlock = threading.Lock()
        self.orin_wlock = threading.Lock()

        # Threads
        self._threads = []

    # ------------- Serial Writers -------------
    def send_motor_pwm(self, m1, m2):
        """Send motor command to STM. M2 inverted as in your code."""
        if not self.stm or not self.stm.is_open:
            return
        m1 = clamp_pwm(m1)
        m2 = clamp_pwm(m2)
        packet = f"M1:{m1},M2:{-m2}\n"
        with self.stm_wlock:
            self.stm.write(packet.encode())

    def send_motor_forward(self, m1, m2):
        """Alternative sign mapping from your code."""
        if not self.stm or not self.stm.is_open:
            return
        m1 = clamp_pwm(m1)
        m2 = clamp_pwm(m2)
        packet = f"M1:{-m1},M2:{+m2}\n"
        with self.stm_wlock:
            self.stm.write(packet.encode())

    def send_telemetry(self):
        """Periodically send <lat,lon,yaw,velocity> to Orin."""
        if not self.orin or not self.orin.is_open:
            return
        while not self.stop_event.is_set():
            with self.state_lock:
                lat = self.state.lat
                lon = self.state.lon
                yaw = self.state.yaw
                vel = self.state.velocity
            payload = f"{lat:.11f},{lon:.11f},{yaw:.3f},{vel:.3f}"
            packet = f"<{payload}>\n"
            try:
                with self.orin_wlock:
                    self.orin.write(packet.encode())
            except Exception as e:
                print(f"[WARN] Telemetry write error: {e}")
            self.stop_event.wait(TELEMETRY_DT)

    # ------------- Orin Command RX -------------
    def recv_orin_commands(self):
        """Line-based receiver: 'lat,lon,cmd,CS' with XOR hex CS of 'lat,lon,cmd'"""
        if not self.orin or not self.orin.is_open:
            return
        while not self.stop_event.is_set():
            try:
                line = self.orin.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                parts = line.split(',')
                if len(parts) < 4:
                    # Not enough fields
                    continue

                data_str = ",".join(parts[:-1])
                cs_hex = parts[-1].upper()
                if compute_checksum_hex(data_str) != cs_hex:
                    # Bad checksum
                    continue

                tlat = float(parts[0])
                tlon = float(parts[1])
                cmd = int(parts[2])

                with self.state_lock:
                    self.state.target_lat = tlat
                    self.state.target_lon = tlon
                    self.state.command = cmd
                # print(f"[CMD] target=({tlat},{tlon}) cmd={cmd}")
            except Exception as e:
                # Keep receiver robust
                # print(f"[WARN] Orin RX error: {e}")
                pass

    # ------------- RTK Reader -------------
    def rtk_reader(self):
        """Continuously read RTK/NMEA and update state."""
        if not self.rtk or not self.rtk.is_open:
            return
        while not self.stop_event.is_set():
            try:
                line = self.rtk.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                fields = line.split(',')

                # Update individual fields if present
                if fields[0] == "#UNIHEADINGA":
                    yaw = parse_uniheadinga(fields)
                    if yaw is not None:
                        with self.state_lock:
                            self.state.yaw = yaw

                elif fields[0] == "$GNGGA":
                    res = parse_gngga(fields)
                    if res:
                        lat, lon, sats, hdop = res
                        with self.state_lock:
                            self.state.lat = lat
                            self.state.lon = lon

                elif fields[0] == "$GNVTG":
                    vel = parse_gnvtg(fields)
                    with self.state_lock:
                        self.state.velocity = vel

            except Exception:
                # Swallow and keep reading
                pass

    # ------------- High-level behaviors -------------
    def control_loop(self):
        """State machine that handles TURN then DRIVE when command==1."""
        # Local copies for filter & PID memory managed in state
        last_loop = time.time()
        while not self.stop_event.is_set():
            loop_start = time.time()
            with self.state_lock:
                cmd = self.state.command
                lat = self.state.lat
                lon = self.state.lon
                yaw = self.state.yaw
                tlat = self.state.target_lat
                tlon = self.state.target_lon

                # PID memory
                prev_filt = self.state.prev_yaw_filt
                e_prev = self.state.err_prev
                e_sum = self.state.err_sum
                left_pwm = self.state.left_pwm
                right_pwm = self.state.right_pwm

            if cmd == 0:
                # Idle
                self.send_motor_pwm(0, 0)
                # Small sleep to avoid busy loop
                self.stop_event.wait(0.05)
                continue

            # TURN phase: rotate-in-place until aligned
            target_brg = bearing_deg(lat, lon, tlat, tlon)
            err_turn = normalize_angle(target_brg - yaw)
            if abs(err_turn) > YAW_TOLERANCE:
                # Proportional turning speed like your yaw_error
                if err_turn < 0:
                    turn_speed = -250 - abs(0.6 * err_turn)
                else:
                    turn_speed = +250 + abs(0.6 * err_turn)
                self.send_motor_pwm(-turn_speed, +turn_speed)
                self.stop_event.wait(LOOP_DT)
                continue  # keep turning until aligned

            # DRIVE phase: go forward with heading correction
            # Smooth yaw
            yaw_f = ALPHA * yaw + (1 - ALPHA) * prev_filt if prev_filt else yaw
            # Error from bearing
            e = normalize_angle(target_brg - yaw_f)

            # Distance check
            dist = haversine(lat, lon, tlat, tlon)
            if dist < DISTANCE_THRESHOLD:
                self.send_motor_pwm(0, 0)
                # Back to idle
                with self.state_lock:
                    self.state.command = 0
                self.stop_event.wait(0.05)
                continue

            # PID terms
            e_sum = clamp(e_sum + (2.0 * e) * LOOP_DT, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
            d_e = (e - e_prev) / LOOP_DT

            u_left = Kp * e + Ki * e_sum + Kd * d_e
            u_right = Kp * e + Ki * e_sum + Kd * d_e

            target_left = BASE_SPEED + u_left
            target_right = BASE_SPEED - u_right

            # Ramp
            if left_pwm < target_left:
                left_pwm = min(left_pwm + SPEED_RAMP, target_left)
            elif left_pwm > target_left:
                left_pwm = max(left_pwm - SPEED_RAMP, target_left)

            if right_pwm < target_right:
                right_pwm = min(right_pwm + SPEED_RAMP, target_right)
            elif right_pwm > target_right:
                right_pwm = max(right_pwm - SPEED_RAMP, target_right)

            # Send
            self.send_motor_forward(left_pwm, right_pwm)

            # Save memory
            with self.state_lock:
                self.state.prev_yaw_filt = yaw_f
                self.state.err_prev = e
                self.state.err_sum = e_sum
                self.state.left_pwm = int(left_pwm)
                self.state.right_pwm = int(right_pwm)

            # Keep loop time
            elapsed = time.time() - loop_start
            remain = max(0.0, LOOP_DT - elapsed)
            self.stop_event.wait(remain)

    # ------------- Lifecycle -------------
    def start(self):
        # Launch threads
        t = threading.Thread(target=self.rtk_reader, name="RTKReader", daemon=True)
        self._threads.append(t); t.start()

        t = threading.Thread(target=self.recv_orin_commands, name="OrinRX", daemon=True)
        self._threads.append(t); t.start()

        t = threading.Thread(target=self.send_telemetry, name="Telemetry", daemon=True)
        self._threads.append(t); t.start()

        t = threading.Thread(target=self.control_loop, name="ControlLoop", daemon=True)
        self._threads.append(t); t.start()

        print("[INFO] All threads started.")

    def stop(self):
        print("[INFO] Stopping...")
        self.stop_event.set()
        # Give threads a moment to exit their waits
        for t in self._threads:
            t.join(timeout=2.0)

        # Stop motors
        try:
            self.send_motor_pwm(0, 0)
        except Exception:
            pass

        # Close serials
        for s in (self.rtk, self.stm, self.orin):
            try:
                if s and s.is_open:
                    s.close()
            except Exception:
                pass
        print("[INFO] Stopped cleanly.")

# =========================
# Entrypoint
# =========================
if __name__ == "__main__":
    ctrl = RobotController(
        stm_port="/dev/uart_converter", stm_baud=9600,
        rtk_port="/dev/rtk", rtk_baud=230400,
        orin_port="/dev/ttyUSB1", orin_baud=115200
    )
    try:
        ctrl.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        ctrl.stop()
