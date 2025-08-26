import time
from .serial_manager import write_orin, get_orin, read_line
from .state import state
from .config import TELEMETRY_INTERVAL

def telemetry_loop():
    """Send telemetry periodically and read incoming commands from Orin."""
    orin = get_orin()
    while True:
        try:
            tx_lat = state.last_known_lat if state.last_known_lat is not None else (state.lat if state.lat is not None else 0.0)
            tx_lon = state.last_known_lon if state.last_known_lon is not None else (state.lon if state.lon is not None else 0.0)
            tx_yaw = state.last_known_yaw if state.last_known_yaw is not None else (state.current_yaw if state.current_yaw is not None else 0.0)
            tx_vel = state.last_known_velocity if state.last_known_velocity is not None else (state.velocity if state.velocity is not None else 0.0)

            # Add placeholders for voltage, current, firingstatus, position if you want to add later
            payload = f"{tx_lat:.11f},{tx_lon:.11f},{tx_yaw},{tx_vel}"
            packet = f"<{payload}>\n"
            write_orin(packet.encode())

            # Non-blocking read window to process incoming Orin commands
            start = time.time()
            read_window = 0.05
            if orin:
                while time.time() - start < read_window:
                    line = read_line(orin)
                    if not line:
                        break
                    parts = line.replace('<','').replace('>','').split(',')
                    try:
                        if len(parts) >= 3:
                            new_lat = float(parts[0])
                            new_lon = float(parts[1])
                            new_cmd = int(float(parts[2]))
                            state.TARGET_LAT = new_lat
                            state.TARGET_LON = new_lon
                            state.COMMAND = new_cmd
                            print(f"[TELEMETRY RX] TARGET updated: {state.TARGET_LAT},{state.TARGET_LON} CMD:{state.COMMAND}")
                        else:
                            print("[TELEMETRY RX] malformed:", line)
                    except Exception as e:
                        print("[TELEMETRY RX] parse error:", e, "line:", line)
                    # keep reading until read_window expires
        except Exception as e:
            print("[TELEMETRY] exception:", e)
        time.sleep(TELEMETRY_INTERVAL)

def start_telemetry_thread():
    import threading
    t = threading.Thread(target=telemetry_loop, daemon=True)
    t.start()
    return t
