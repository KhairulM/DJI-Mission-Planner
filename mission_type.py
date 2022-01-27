import enum


# Class definition of a Mission type
class MissionType(enum.Enum):
    up = 1
    down = 2
    left = 3
    right = 4
    rotate = 5
    up_to = 6
    wait_for_cv = 7
    takeoff = 8
    rth = 9
    land = 10
