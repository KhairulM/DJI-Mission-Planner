import enum


# Class definition of a Mission type
class MissionType(enum.Enum):
    up_to = 1
    down_to = 2
    up = 3
    down = 4
    left = 5
    right = 6
    rotate = 7
    wait_for_cv = 8
    takeoff = 9
    rth = 10
    land = 11
    align_with_barcode = 12
