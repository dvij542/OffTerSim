import pyproj


def determine_utm_zone(longitude):
    """Determine the UTM zone for a given longitude"""
    return int((longitude + 180) / 6) + 1

def lla_to_utm(lat, lon):
    """Convert LLA to UTM coordinates"""
    zone = determine_utm_zone(lon)
    utm_proj = pyproj.Proj(proj='utm', zone=zone, ellps='WGS84')
    x, y = utm_proj(lon, lat)
    return x, y