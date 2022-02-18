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

        url = "http://" + self.host + ":6868" + "/api/v1/config/" + self.missionId
        r = requests.get(url)

        if r.status_code != 200:
            raise Exception("Response status code: %d" % r.status_code)

        missionConfiguration = r.json()

        # with open("./mission-configuration.example.json") as f:
        #     missionConfiguration = json.load(f)

        maxAlt = float(missionConfiguration["max_altitude"])
        minAlt = float(missionConfiguration["min_altitude"])
        missionSpeed = float(missionConfiguration["mission_speed"])
        sweepConfig = missionConfiguration["sweep_config"]
        endPoint = missionConfiguration["end_point"]
        turningPoint = missionConfiguration["turning_point"]

        defaultTransitionMission = MissionType[missionConfiguration["orientation"]]

        missions = []

        # add takeoff
        missions.append(Mission(MissionType.takeoff))

        fromBottom = True

        for index, sweep in enumerate(sweepConfig):
            rackSize = sweep["rack_size"]

            levelHeights = self.transformLevelHeights(rackSize["level_height"], maxAlt)

            if sweep["scan"]:
                for i, levelHeight in (
                    enumerate(levelHeights)
                    if fromBottom
                    else enumerate(reversed(levelHeights))
                ):
                    missions.append(Mission(MissionType.wait_for_alt, [levelHeight]))

                    missions.append(
                        Mission(
                            MissionType.publish_rack_id,
                            [
                                "%s-%d"
                                % (
                                    sweep["rack_id"],
                                    i + 1 if fromBottom else len(levelHeights) - i,
                                )
                            ],
                        )
                    )

                    # missions.append(
                    #     Mission(
                    #         MissionType.align_with_barcode,
                    #         [
                    #             "%s-%d"
                    #             % (
                    #                 sweep["rack_id"],
                    #                 i + 1 if fromBottom else len(levelHeights) - i,
                    #             )
                    #         ],
                    #     )
                    # )

                    # isHighestLevel = (
                    #     (i == len(levelHeights) - 1) if fromBottom else (i == 0)
                    # )

                    # missions.append(
                    #     Mission(
                    #         MissionType.wait_for_cv,
                    #         [isHighestLevel],
                    #     )
                    # )

                fromBottom = not fromBottom

            if sweep["rack"] != endPoint:
                if sweep["rack"] == turningPoint:
                    missions.append(Mission(MissionType.rotate, [180.0]))
                else:
                    missions.append(
                        Mission(
                            defaultTransitionMission,
                            [
                                rackSize["width"] / 2
                                + sweepConfig[index + 1]["rack_size"]["width"] / 2
                            ],
                        )
                    )

                missions.append(
                    Mission(
                        MissionType.publish_rack_id,
                        [
                            "%s-%d"
                            % (
                                sweepConfig[index + 1]["rack_id"],
                                1
                                if fromBottom
                                else len(
                                    sweepConfig[index + 1]["rack_size"]["level_height"]
                                ),
                            )
                        ],
                    )
                )

        # land after finishing missions
        missions.append(Mission(MissionType.land))

        if self.verbose:
            print("MissionLoader: load:", missions)

        return {
            "missions": missions,
            "max_altitude": maxAlt,
            "min_altitude": minAlt,
            "mission_speed": missionSpeed,
        }

    def transformLevelHeights(self, levelHeights, maxAlt):
        newLevelHeights = []
        sum = 0

        for height in levelHeights:
            alt = min(sum + height / 2, maxAlt)
            newLevelHeights.append(alt)
            sum += height

        return newLevelHeights
