import time
import threading
from .serial_manager import initialize_serial 
from .telemetry import start_telemetry_thread
from .autonomous import auto_turn, auto_move
from .state import state
from .motor_control import send_stop
from .serial_manager import start_orin_reader
from .config import COMMAND, TARGET_LAT, TARGET_LON
from .autonomous import auto_turn, auto_move
from .motor_control import send_motor_command


orin_serial = initialize_serial('/dev/ttyUSB1', 115200)  # Orin link
start_orin_reader(orin_serial)

def main():
    # Initialize serial ports
    rtk, stm, orin = initialize_serial()
    print("[MAIN] Serial init done.")

    # Start telemetry thread (it will read/write Orin)
    start_telemetry_thread()
    print("[MAIN] Telemetry started.")

    # Mission runner thread
    def mission_runner():
        print("[MAIN] mission runner started.")
        while True:
            if COMMAND == 1 and TARGET_LAT and TARGET_LON:
                auto_turn(TARGET_LAT, TARGET_LON)
                auto_move(TARGET_LAT, TARGET_LON)
            elif COMMAND == 0:
                send_motor_command(0, 0)

    t = threading.Thread(target=mission_runner, daemon=True)
    t.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[MAIN] KeyboardInterrupt: stopping.")
        send_stop()
        # close serials cleanly if needed (in serial_manager module)
        try:
            rtk.close()
        except Exception:
            pass
        try:
            stm.close()
        except Exception:
            pass
        try:
            orin.close()
        except Exception:
            pass
        print("[MAIN] Exiting.")

if __name__ == "__main__":
    main()
