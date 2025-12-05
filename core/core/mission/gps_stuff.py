from math import radians, cos, sin, asin, sqrt, atan2, degrees
from pyproj import Geod

# Anggap black box :0
# https://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points
def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance in meters between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles. Determines return value units.
    return c * r * 1000 # return in meters


# Returns positive theta if RIGHT, negative theta if LEFT
def turner(ship_coor, heading, goal):
    goal_v_x = goal[0] - ship_coor[0]
    goal_v_y = goal[1] - ship_coor[1]

    ship_v_x = cos(radians(heading))
    ship_v_y = sin(radians(heading))
    right_heading = heading - 90 if heading >= 90 else heading + 270
    right_v_x = cos(radians(right_heading))
    right_v_y = sin(radians(right_heading))

    magnitude = ship_v_x * goal_v_x + ship_v_y * goal_v_y
    sign_dot_product = ship_v_x * right_v_x + ship_v_y * right_v_y
    sign = 1 if sign_dot_product >= 0 else -1
    return magnitude * sign * -1.0 

def calc_dsc(target, heading):
    dsc = target - heading
    dsc = dsc if abs(dsc) <= 180 else (360 - abs(dsc)) * (-1 if dsc > 0 else 1)
    
    return dsc

def calc_turn(target, heading):
    diff = (target - heading + 540) % 360 - 180
    return diff

geodesic = Geod(ellps='WGS84')
def find_deg(lat1, lon1, lat2, lon2, heading):
    fwd_azimuth, _, _ = geodesic.inv(radians(lon1), radians(lat1), radians(lon2), radians(lat2))
    fwd_azimuth %= 360
    return (fwd_azimuth - heading + 180 ) % 360 - 180
