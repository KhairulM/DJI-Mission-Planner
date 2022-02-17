import enum


# Class definition of a Mission type
class MissionType(enum.Enum):
    wait_for_alt = 1
    up = 2
    down = 3
    left = 4
    right = 5
    rotate = 6
    wait_for_cv = 7
    takeoff = 8
    rth = 9
    land = 10
    align_with_barcode = 11
    publish_rack_id = 12
