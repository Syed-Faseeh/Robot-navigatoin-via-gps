import math

def safe_float(s):
    try:
        return float(s)
    except Exception:
        return None

def gngga_to_decimal(nmea_lat, nmea_lat_dir, nmea_lon, nmea_lon_dir):
    """Convert NMEA lat/lon (ddmm.mmmm / dddmm.mmmm) to decimal degrees or (None,None)."""
    try:
        if nmea_lat and len(nmea_lat) >= 4:
            lat_degrees = int(nmea_lat[:2])
            lat_minutes = float(nmea_lat[2:])
            latitude = lat_degrees + (lat_minutes / 60.0)
            if nmea_lat_dir == 'S':
                latitude = -latitude
        else:
            return None, None

        if nmea_lon and len(nmea_lon) >= 5:
            lon_degrees = int(nmea_lon[:3])
            lon_minutes = float(nmea_lon[3:])
            longitude = lon_degrees + (lon_minutes / 60.0)
            if nmea_lon_dir == 'W':
                longitude = -longitude
        else:
            return None, None

        return latitude, longitude
    except Exception:
        return None, None

def get_current_location(fields):
    """Parse $GNGGA fields -> (lat, lon, satellites, hdop) or (None, None, None, None)"""
    try:
        nmea_lat = fields[2]
        nmea_lat_dir = fields[3]
        nmea_lon = fields[4]
        nmea_lon_dir = fields[5]
        satellites = fields[7] if len(fields) > 7 else None
        hdop = fields[8] if len(fields) > 8 else None
        lat_d, lon_d = gngga_to_decimal(nmea_lat, nmea_lat_dir, nmea_lon, nmea_lon_dir)
        return lat_d, lon_d, satellites, hdop
    except Exception:
        return None, None, None, None

def get_yaw(fields):
    """Parse heading from #UNIHEADINGA (defensive)"""
    try:
        # original code indexed 12; keep defensive check
        if len(fields) > 12:
            return safe_float(fields[12])
        return None
    except Exception:
        return None

def get_velocity_data(fields):
    """Parse velocity from $GNVTG (defensive)"""
    try:
        if len(fields) >= 3:
            return safe_float(fields[-3])
        return None
    except Exception:
        return None

def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def haversine(lat1, lon1, lat2, lon2):
    try:
        R = 6371000  # meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c
    except Exception:
        return float('inf')

def calculate_bearing(lat1, lon1, lat2, lon2):
    try:
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlambda = math.radians(lon2 - lon1)
        x = math.sin(dlambda) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
        bearing = math.atan2(x, y)
        return (math.degrees(bearing) + 360) % 360
    except Exception:
        return None

def yaw_error(target, current):
    try:
        if target is None or current is None:
            return None, 0
        error = target - current
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        if error < 0:
            turn_speed = -250 - abs(0.6 * error)
        else:
            turn_speed = 250 + abs(0.6 * error)
        return error, turn_speed
    except Exception:
        return None, 0
