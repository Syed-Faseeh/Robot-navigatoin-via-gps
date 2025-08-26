# Configuration and tunable constants
Kp_1 = 8.0
Ki_1 = 0.8
Kd_1 = 0.0

Kp_2 = 8.0
Ki_2 = 0.8
Kd_2 = 0.0

ALPHA = 0.3
SPEED_RAMP = 5
BASE_SPEED = 750
LOOP_DT = 0.1  # 10 Hz control loop
TELEMETRY_INTERVAL = 0.5  # seconds

YAW_TOLERANCE = 1  # degrees (alignment threshold)
DISTANCE_THRESHOLD = 0.5  # meters (arrival threshold)

# Serial device paths — change these to match your system
STM_PORT = '/dev/uart_converter'   # STM serial (motor controller)
RTK_PORT = '/dev/rtk'              # RTK GNSS
ORIN_PORT = '/dev/rs232tousb'      # Radio / Orin link

# Baud rates
STM_BAUD = 115200
RTK_BAUD = 230400
ORIN_BAUD = 115200

TARGET_LAT = None
TARGET_LON = None
COMMAND = 0

