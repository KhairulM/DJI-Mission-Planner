# Class definition of a Mission command
from mission_type import MissionType


class Mission:
    def __str__(self) -> str:
        return "Mission " + self.typeString + " " + self.argument

    def __repr__(self) -> str:
        return "<Mission type:%s arg:%s>" % (self.typeString, str(self.argument))

    def __init__(self, type, argument=[]):
        self.type = type
        self.typeString = "undefined"
        self.argument = argument

        if self.type == MissionType.wait_for_alt:
            self.typeString = "wait for alt"
        elif self.type == MissionType.up:
            self.typeString = "up"
        elif self.type == MissionType.down:
            self.typeString = "down"
        elif self.type == MissionType.right:
            self.typeString = "right"
        elif self.type == MissionType.left:
            self.typeString = "left"
        elif self.type == MissionType.rotate:
            self.typeString = "rotate"
        elif self.type == MissionType.wait_for_cv:
            self.typeString = "wait for cv"
        elif self.type == MissionType.takeoff:
            self.typeString = "takeoff"
        elif self.type == MissionType.rth:
            self.typeString = "rth"
        elif self.type == MissionType.land:
            self.typeString = "land"
        elif self.type == MissionType.align_with_barcode:
            self.typeString = "align with barcode"
