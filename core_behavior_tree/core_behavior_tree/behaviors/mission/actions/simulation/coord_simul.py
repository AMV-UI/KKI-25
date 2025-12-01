from math import *

def calc_dsc(target, heading):
    dsc = target - heading
    dsc = dsc if abs(dsc) <= 180 else (360 - abs(dsc)) * (-1 if dsc > 0 else 1)
    
    return dsc

def find_deg(cur_lat, cur_lon, tar_lat, tar_lon, cur_head):
    delta_y = tar_lat - cur_lat
    delta_x = tar_lon - cur_lon

    if delta_x == 0:
        return 180 * (delta_y < 0)

    theta = degrees(atan(abs(delta_y) / abs(delta_x)))

    deg = 0
    if(delta_x > 0):
        if(delta_y > 0):
            deg = 90 - theta
        else:
            deg = 90 + theta
    else:
        if(delta_y > 0):
            deg = (90 - theta) + 180
        else:
            deg = theta + 270

    return calc_dsc(int(deg), cur_head)