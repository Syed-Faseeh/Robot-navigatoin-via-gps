import serial
import time


# Function to initialize serial connection
def initialize_serial():
    """Initialize the serial connection."""
    try:
        ser = serial.Serial(
            port='/dev/ttyUSB0', 
            baudrate=230400,        
            timeout=1
        )
        return ser
    except serial.SerialException as e:
        print(f"Error initializing serial port: {e}")
        return None

ser = None  

# Call the test function to verify NMEA conversion

def get_velocity_data(data):
    try:
        if data[0] == "$GNVTG":
            velocity = data[-3]
            print(f"Velocity = {velocity}")

    except Exception as e:
        print(e)
        pass
def get_yaw_data(sentence):
    try:
        if sentence[0]==("#UNIHEADINGA"):
            yaw=sentence[12]
            yaw=str(yaw)
            print("yaw:",yaw)
            time.sleep(0.5)

            return yaw
    except Exception as e:
            print(e)
            pass
    
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
        # Longitude conversion
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
    

def get_location_data(data):
    try:
        if data[0] == "$GNGGA":
            nmea_lat=data[2]
            nmea_lat_dir=data[3]
            nmea_lon=data[4]
            nmea_lon_dir=data[5]
            latitude,longitude=gngga_to_decimal(nmea_lat, nmea_lat_dir, nmea_lon, nmea_lon_dir)
            location=f"{longitude},{latitude}"
            print(location)
    except Exception as e:
        print(e)
        pass
      
                                          
def process_serial_data():
    """Read data from the serial port and process it in separate threads."""
    global ser

    while True:       
        if ser is not None and ser.is_open:
            try:
                response = ser.readline().decode('ascii').strip()
                data = response.split(',')

                if (data):
                    get_velocity_data(data)
                    get_yaw_data(data)
                    get_location_data(data)

            except serial.SerialException as e:
                print(f"Serial port error: {e}")
                ser.close()
                ser = initialize_serial()
                time.sleep(1)  # Wait a moment before trying again
        else:
            # If serial port is not open or has been closed, try to reinitialize
            ser = initialize_serial()
            time.sleep(1)  # Wait a moment before trying again

process_serial_data()
