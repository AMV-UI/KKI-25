from math import sin, cos, radians

# Returns negative theta if RIGHT, positive theta if LEFT
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
    sign = -1 if sign_dot_product >= 0 else 1
    return magnitude * sign * -1.0  # Negative for RIGHT, Positive for LEFT


print(turner((0, 0), 0, (-1, -1)))    # Expect positive (RIGHT)