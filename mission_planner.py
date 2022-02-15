import argparse
import paho.mqtt.client as mqtt
from mission_loader import MissionLoader
from mission_executor import MissionExecutor

TOPIC_MISSION_PLANNER_START = "mission-planner/start"
TOPIC_MISSION_PLANNER_START_RESULT = "mission-planner/start/result"
TOPIC_MISSION_PLANNER_PAUSE = "mission-planner/pause"
TOPIC_MISSION_PLANNER_PAUSE_RESULT = "mission-planner/pause/result"
TOPIC_MISSION_PLANNER_RESTART = "mission-planner/restart"
TOPIC_MISSION_PLANNER_RESTART_RESULT = "mission-planner/restart/result"
TOPIC_MISSION_PLANNER_SHUTDOWN = "mission-planner/shutdown"
TOPIC_MISSION_PLANNER_SHUTDOWN_RESULT = "mission-planner/shutdown/result"
TOPIC_MISSION_PLANNER_MISSION_ID = "mission-planner/mission-id"
TOPIC_DJI_STATUS_CONNECTION = "dji/status/connection"
# TOPIC_DJI_STATUS_FLIGHT_MODE = "dji/status/flight-mode"
TOPIC_DJI_STATUS_FLIGHT_CONTROL = "dji/status/flight-control"
TOPIC_DJI_CONTROL_LAND = "dji/control/land"
TOPIC_DJI_CONTROL_LAND_RESULT = "dji/control/land/result"


class MissionPlanner:
    def __init__(self, host, port, username, password, missionId, sendFreq, verbose):
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.missionId = missionId
        self.verbose = verbose
        self.mqttClient = mqtt.Client()
        self.missionLoader = MissionLoader(host, missionId, verbose)
        self.missionExecutor = MissionExecutor(username, password, sendFreq, verbose)

        self.droneConnectionStatus = False
        self.droneFlightControlStatus = False
        self.droneFlightMode = ""

        self.isMissionLoaded = False
        self.isStartMission = False
        self.isPauseMission = False

        self.currentMissionIndex = 0
        self.MISSIONS = []

        if self.verbose:
            print("MissionPlanner: init: initializing mqtt")

        self.mqttClient.max_inflight_messages_set(50)
        self.mqttClient.username_pw_set(username, password)

        self.mqttClient.message_callback_add(
            TOPIC_DJI_STATUS_CONNECTION, self.onDJIConnectionStatus
        )
        # self.mqttClient.message_callback_add(
        #     TOPIC_DJI_STATUS_FLIGHT_MODE, self.onDJIFlightModeStatus)
        self.mqttClient.message_callback_add(
            TOPIC_DJI_STATUS_FLIGHT_CONTROL, self.onDJIFlightControlStatus
        )
        self.mqttClient.message_callback_add(
            TOPIC_MISSION_PLANNER_START, self.onMissionPlannerStart
        )
        self.mqttClient.message_callback_add(
            TOPIC_MISSION_PLANNER_SHUTDOWN, self.onMissionPlannerShutdown
        )
        self.mqttClient.message_callback_add(
            TOPIC_MISSION_PLANNER_PAUSE, self.onMissionPlannerPause
        )
        self.mqttClient.message_callback_add(
            TOPIC_MISSION_PLANNER_RESTART, self.onMissionPlannerRestart
        )

    def start(self):
        first = True
        missionFailCount = 0

        self.connect()

        while True:
            if (
                self.isPauseMission
                or not self.isStartMission
                or not self.mqttClient.is_connected()
            ):
                continue

            if first:
                self.loadMission()
                first = False

            # DEFAULT ACTION TO TAKE IF MISSION FAILED MULTIPLE TIMES
            if missionFailCount >= 3:
                self.pauseMissionExecution(True)
                self.startLanding()
                missionFailCount = 0
                continue

            if self.currentMissionIndex >= len(self.MISSIONS):
                if self.verbose:
                    print("MissionPlanner: mission finished")

                self.isStartMission = False
                self.mqttClient.publish(
                    TOPIC_MISSION_PLANNER_START_RESULT, "completed", 1, True
                )
                continue

            result = self.missionExecutor.execute(
                self.MISSIONS[self.currentMissionIndex]
            )

            if self.verbose:
                print("MisionPlanner: mission execution result:", result)

            if result == 0:
                self.currentMissionIndex += 1
                missionFailCount = 0
            elif result == -1:
                missionFailCount += 1

    def connect(self):
        if self.verbose:
            print("MissionPlanner: connect: connecting mqtt")

        self.mqttClient.on_connect = self.onMqttConnect
        self.mqttClient.connect(self.host, self.port)
        self.mqttClient.loop_start()

        self.missionExecutor.connect(self.host, self.port)

    def disconnect(self):
        self.mqttClient.on_disconnect = self.onMqttDisconnect
        self.mqttClient.disconnect()
        self.mqttClient.loop_stop()

        self.missionExecutor.disconnect()

    def loadMission(self):
        if self.verbose:
            print("MissionPlanner: loadMission: loading mission")

        self.isMissionLoaded = False

        try:
            missionConfig = self.missionLoader.load()
            self.MISSIONS = missionConfig["missions"]

            self.missionExecutor.setMaxAltitude(missionConfig["max_altitude"])
            self.missionExecutor.setMinAltitude(missionConfig["min_altitude"])
            self.missionExecutor.setMissionSpeed(missionConfig["mission_speed"])

            self.isMissionLoaded = True
            self.mqttClient.publish(
                TOPIC_MISSION_PLANNER_MISSION_ID, self.missionId, 1, True
            )

            if self.verbose:
                print("MissionPlanner: loadMission: mission loaded")
        except Exception as e:
            print("[ERR] MissionPlanner: loadMission exception:", str(e))

    def onMqttConnect(self, client, userdata, flags, rc):
        if self.verbose:
            print("MissionPlanner: onMqttConnect:", mqtt.connack_string(rc))

        self.mqttClient.subscribe(
            [
                (TOPIC_DJI_STATUS_CONNECTION, 1),
                # (TOPIC_DJI_STATUS_FLIGHT_MODE, 1),
                (TOPIC_DJI_STATUS_FLIGHT_CONTROL, 1),
                (TOPIC_MISSION_PLANNER_START, 1),
                (TOPIC_MISSION_PLANNER_PAUSE, 1),
                (TOPIC_MISSION_PLANNER_RESTART, 2),
                (TOPIC_MISSION_PLANNER_SHUTDOWN, 1),
                (TOPIC_DJI_CONTROL_LAND_RESULT, 2),
            ]
        )

    def onMqttDisconnect(self, client, userdata, rc):
        if self.verbose:
            print("MissionPlanner: onMqttDisconnect:", mqtt.connack_string(rc))

    def pauseMissionExecution(self, pause):
        if self.verbose:
            print("MissionPlanner: pauseMissionExecution:", pause)

        self.isPauseMission = pause

        self.mqttClient.publish(
            TOPIC_MISSION_PLANNER_PAUSE_RESULT,
            "paused" if pause else "unpaused",
            1,
            True,
        )

    def startLanding(self):
        if self.verbose:
            print("MissionPlanner: startLanding: true")

        self.mqttClient.publish(TOPIC_DJI_CONTROL_LAND, "true", 2)

    def onDJIConnectionStatus(self, client, userdata, msg):
        if self.verbose:
            print("MissionPlanner: onDJIConnectionStatus:", msg.payload.decode())

        self.droneConnectionStatus = msg.payload.decode().lower() == "true"
        self.pauseMissionExecution(not self.droneConnectionStatus)

    # def onDJIFlightModeStatus(self, client, userdata, msg):
    #     if (self.verbose):
    #         print("MissionPlanner: onDJIFlightModeStatus:", msg.payload.decode())

    #     self.droneFlightMode = msg.payload.decode()

    def onDJIFlightControlStatus(self, client, userdata, msg):
        if self.verbose:
            print("MissionPlanner: onDJIFlightControlStatus:", msg.payload.decode())

        self.droneFlightControlStatus = msg.payload.decode().lower() == "true"
        self.pauseMissionExecution(not self.droneFlightControlStatus)

    def onMissionPlannerStart(self, client, userdata, msg):
        if self.verbose:
            print("MissionPlanner: onMissionPlannerStart:", msg.payload.decode())

        self.isStartMission = msg.payload.decode().lower() == "true"

        self.mqttClient.publish(
            TOPIC_MISSION_PLANNER_START_RESULT,
            "started" if self.isStartMission else "failed",
            1,
            True,
        )

    def onMissionPlannerPause(self, client, userdata, msg):
        if self.verbose:
            print("MissionPlanner: onMissionPlannerPause:", msg.payload.decode())

        self.pauseMissionExecution(msg.payload.decode().lower() == "true")

    def onMissionPlannerRestart(self, client, userdata, msg):
        if self.verbose:
            print("MissionPlanner: onMissionPlannerRestart:", msg.payload.decode())

        if msg.payload.decode().lower() == "false":
            client.publish(TOPIC_MISSION_PLANNER_RESTART_RESULT, "failed", 2)
            return

        self.currentMissionIndex = 0
        self.pauseMissionExecution(False)
        self.isStartMission = False

        client.publish(TOPIC_MISSION_PLANNER_RESTART_RESULT, "restarted", 2)

    def onMissionPlannerShutdown(self, client, userdata, msg):
        if self.verbose:
            print("MissionPlanner: onMissionPlannerShutdown:", msg.payload.decode())

        if msg.payload.decode().lower() == "true":
            self.mqttClient.publish(TOPIC_MISSION_PLANNER_SHUTDOWN_RESULT, "stopped", 1)
            self.disconnect()
            quit()

        self.mqttClient.publish(TOPIC_MISSION_PLANNER_SHUTDOWN_RESULT, "failed", 1)


parser = argparse.ArgumentParser()
parser.description = "Mission Planner for Panasonic DJI Drone"
parser.add_argument(
    "--host",
    help="Hostname for MQTT broker and the backend server",
    type=str,
    default="localhost",
)
parser.add_argument(
    "-p", help="Port number for MQTT broker", dest="port", type=int, default=1883
)
parser.add_argument(
    "-u",
    help="Username for MQTT authentication",
    dest="username",
    type=str,
    default="admin",
)
parser.add_argument(
    "-P",
    help="Password for MQTT authentication",
    dest="password",
    type=str,
    default="1234",
)
parser.add_argument(
    "-id",
    help="ID of the mission to be loaded",
    dest="missionId",
    type=str,
    default="latest",
)
parser.add_argument(
    "-f",
    help="The frequency for the execcutor to send control data (5 - 25)",
    dest="freq",
    type=int,
    default=10,
)
parser.add_argument(
    "-v", help="Set the logging level to verbose", dest="verbose", action="store_true"
)
parser.set_defaults(verbose=True)
args = parser.parse_args()

if __name__ == "__main__":
    mp = MissionPlanner(
        args.host,
        args.port,
        args.username,
        args.password,
        args.missionId,
        args.freq,
        args.verbose,
    )
    mp.start()
