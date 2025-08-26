from .serial_manager import get_stm
from .config import BASE_SPEED, SPEED_RAMP

def clamp_pwm(value, min_value=-1000, max_value=1000):
    try:
        return max(min_value, min(max_value, int(value)))
    except Exception:
        return 0

def ramp_speed(current, target):
    if current < target:
        return min(current + SPEED_RAMP, target)
    elif current > target:
        return max(current - SPEED_RAMP, target)
    return current

def send_motor_command(left, right):
    """
    left, right: human-friendly (positive = forward).
    When sending to STM, we invert right (STM expects M2:-right).
    """
    left_s = clamp_pwm(left)
    right_s = clamp_pwm(right)
    packet = f"M1:{left_s},M2:{-right_s}\n"
    print("[SEND STM]", packet.strip())
    stm = get_stm()
    try:
        if stm and stm.is_open:
            stm.write(packet.encode())
    except Exception as e:
        print(f"[WARN] STM write failed: {e}")

def send_stop():
    send_motor_command(0, 0)
