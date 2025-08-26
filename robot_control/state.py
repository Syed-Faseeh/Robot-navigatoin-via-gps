from .config import BASE_SPEED

class _State:
    def __init__(self):
        # Live measurement values (None until valid)
        self.lat = None
        self.lon = None
        self.last_known_lat = None
        self.last_known_lon = None

        self.current_yaw = None
        self.last_known_yaw = None

        self.velocity = None
        self.last_known_velocity = None

        # Target and command coming from Orin


        # Motor PWM values (human-friendly: positive = forward)
        self.left_pwm = BASE_SPEED
        self.right_pwm = BASE_SPEED

state = _State()
