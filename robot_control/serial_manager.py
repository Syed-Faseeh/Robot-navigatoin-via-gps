import serial
import threading
import time
import config

# module-level serial objects and lock for Orin writes
_rtk = None
_stm = None
_orin = None
_orin_lock = threading.Lock()


def initialize_serial():
    """Open and store serial ports. Returns (rtk, stm, orin)"""
    global _rtk, _stm, _orin
    try:
        _stm = serial.Serial(config.STM_PORT, config.STM_BAUD, timeout=0.2)
    except Exception as e:
        print(f"[WARN] STM serial init failed: {e}")
        _stm = None
    try:
        _rtk = serial.Serial(config.RTK_PORT, config.RTK_BAUD, timeout=0.5)
    except Exception as e:
        print(f"[WARN] RTK serial init failed: {e}")
        _rtk = None
    try:
        _orin = serial.Serial(config.ORIN_PORT, config.ORIN_BAUD, timeout=0.2)
    except Exception as e:
        print(f"[WARN] Orin serial init failed: {e}")
        _orin = None
    # small pause to let serial stabilize
    time.sleep(0.1)
    return _rtk, _stm, _orin

def get_rtk():
    return _rtk

def get_stm():
    return _stm

def get_orin():
    return _orin

def write_orin(data: bytes):
    """Thread-safe write to Orin serial (or fallback to print)."""
    with _orin_lock:
        try:
            if _orin and _orin.is_open:
                _orin.write(data)
            else:
                # no Orin: print for debugging
                print("[ORIN OUT]", data.decode(errors='ignore').strip())
        except Exception as e:
            print(f"[WARN] Orin write failed: {e}")
            
def read_from_orin(orin_serial):
    while True:
        try:
            line = orin_serial.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue

            parts = line.split(',')
            if len(parts) >= 3:
                config.TARGET_LAT = float(parts[0])
                config.TARGET_LON = float(parts[1])
                config.COMMAND = int(parts[2])
        except Exception as e:
            print("Error reading from Orin:", e)
def start_orin_reader(orin_serial):
    thread = threading.Thread(target=read_from_orin, args=(orin_serial,), daemon=True)
    thread.start()


def read_line(ser):
    """Safe readline wrapper; returns decoded string or None."""
    try:
        if ser and ser.is_open and ser.in_waiting > 0:
            return ser.readline().decode('ascii', errors='ignore').strip()
    except Exception:
        return None
    return None
