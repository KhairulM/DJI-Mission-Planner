# Class definition of a Mission command
from mission_type import MissionType


class Mission:
    def __init__(self, type, argument):
        self.type = type
        self.typeString = "undefined"
        self.argument = argument
        self.controlData = [0, 0, 0, 0]

        if (self.type == MissionType.up):
            self.controlData = [0.0, 0.0, 0.0, abs(self.argument[0])]
            self.typeString = "up"
        elif (self.type == MissionType.down):
            self.typeString = "down"
            self.controlData = [0.0, 0.0, 0.0, -abs(self.argument[0])]
        elif (self.type == MissionType.right):
            self.typeString = "right"
            # TODO
        elif (self.type == MissionType.left):
            self.typeString = "left"
            # TODO
        elif (self.type == MissionType.rotate):
            self.typeString = "rotate"
            # TODO
        elif (self.type == MissionType.up_to):
            self.typeString = "up to"
            # TODO
        elif (self.type == MissionType.wait_for_cv):
            self.typeString = "wait_for_cv"
        elif (self.type == MissionType.takeoff):
            self.typeString = "takeoff"
        elif (self.type == MissionType.rth):
            self.typeString = "rth"
        elif (self.type == MissionType.land):
            self.typeString = "land"
