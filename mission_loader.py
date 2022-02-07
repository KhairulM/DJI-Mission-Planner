import requests
import json
from mission import Mission
from mission_type import MissionType


class MissionLoader:
    def __init__(self, host, missionId, verbose):
        self.missionId = missionId
        self.verbose = verbose
        self.host = host

    # Translate mission configuration to array of mission
    # Return: array of Mission, mission max altitude, and mission speed
    def load(self):
        missionConfiguration = None

        # url = "http://" + self.host + ":6868" + "/api/v1/config/" + self.missionId
        # r = requests.get(url)

        # if (r.status_code != 200):
        #     raise "Response status code: %d" % r.status_code

        # missionConfiguration = r.json()

        with open("./mission-configuration.example.json") as f:
            missionConfiguration = json.load(f)

        maxAlt = float(missionConfiguration["max_altitude"])
        missionSpeed = float(missionConfiguration["mission_speed"])
        sweepConfig = missionConfiguration["sweep_config"]
        rackSize = missionConfiguration["rack_size"]

        sweepBool = []
        for rack in range(1, sweepConfig[-1]+1):
            sweepBool.append(rack in sweepConfig)

        missions = []

        # add takeoff
        missions.append(Mission(MissionType.takeoff))

        fromBottom = True
        rackIdIdx = 0

        for rack, isScan in enumerate(sweepBool):
            isTransitionToNextRack = rack < len(sweepBool)-1
            levelHeights = self.transformLevelHeights(
                rackSize[rack]["level_height"])

            if (isScan):
                for i, levelHeight in enumerate(levelHeights) if fromBottom else enumerate(reversed(levelHeights)):
                    missions.append(
                        Mission(MissionType.up_to, [levelHeight]))

                    missions.append(Mission(MissionType.align_with_barcode, [
                                    "%s-%d" % (missionConfiguration["rack_ids"][rackIdIdx], i+1 if fromBottom else len(levelHeights)-i)]))
                    missions.append(Mission(MissionType.wait_for_cv, [
                                    "%s-%d" % (missionConfiguration["rack_ids"][rackIdIdx], i+1 if fromBottom else len(levelHeights)-i)]))

                    if (i == len(levelHeights) - 1 and isTransitionToNextRack):
                        missions.append(
                            Mission(MissionType.right, [rackSize[rack]["width"] / 2 + rackSize[rack+1]["width"]/2]))

                fromBottom = False
                rackIdIdx += 1
            else:
                if (isTransitionToNextRack):
                    missions.append(
                        Mission(MissionType.right, [rackSize[rack]["width"] / 2 + rackSize[rack+1]["width"]/2]))

        # rth after finishing missions
        missions.append(Mission(MissionType.rth))
        missions.append(Mission(MissionType.land))

        if (self.verbose):
            print("MissionLoader: load:", missions)

        return {
            "missions": missions,
            "max_altitude": maxAlt,
            "mission_speed": missionSpeed
        }

    def transformLevelHeights(self, levelHeights):
        newLevelHeights = []
        sum = 0

        for height in levelHeights:
            sum += height
            newLevelHeights.append(sum - height/2)

        return newLevelHeights
