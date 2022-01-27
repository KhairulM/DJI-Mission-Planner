import argparse
import paho.mqtt.client as mqtt
from mission_loader import MissionLoader
from mission_executor import MissionExecutor


class MissionPlanner:
    def __init__(self, host, port, username, password, missionId, sendFreq, verbose):
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.missionId = missionId
        self.verbose = verbose
        self.mqttClient = mqtt.Client()
        self.missionLoader = MissionLoader(missionId, verbose)
        self.missionExecutor = MissionExecutor(
            username, password, sendFreq, verbose)

        self.droneConnectionStatus = False
        self.droneFlightControlStatus = False
        self.droneFlightMode = ""

        self.isMissionLoaded = False
        self.isStartMission = False
        self.isPauseMission = False

        self.currentMissionIndex = 0
        self.MISSIONS = []

        if (self.verbose):
            print("MissionPlanner: init: initializing mqtt")

        self.mqttClient.max_inflight_messages_set(50)
        self.mqttClient.username_pw_set(username, password)

        self.mqttClient.message_callback_add(
            "dji/status/connection", self.onDJIConnectionStatus)
        self.mqttClient.message_callback_add(
            "dji/status/flight-mode", self.onDJIFlightModeStatus)
        self.mqttClient.message_callback_add(
            "dji/status/flight-control", self.onDJIFlightControlStatus)
        self.mqttClient.message_callback_add(
            "mission-planner/start", self.onMissionPlannerStart)
        self.mqttClient.message_callback_add(
            "mission-planner/shutdown", self.onMissionPlannerShutdown)
        self.mqttClient.message_callback_add(
            "mission-planner/pause", self.onMissionPlannerPause)

    def start(self):
        missionFailCount = 0

        self.connect()
        self.loadMission()

        while True:
            if (self.isPauseMission or
                not self.isStartMission or
                not self.isMissionLoaded or
                    not self.mqttClient.is_connected()):
                continue

            result = self.missionExecutor.execute(
                self.MISSIONS[self.currentMissionIndex])

            if result == 0:
                self.currentMissionIndex += 1
                missionFailCount = 0
                continue
            elif result == -1:
                missionFailCount += 1

            # DEFAULT ACTION TO TAKE IF MISSION FAILED MULTIPLE TIMES
            if missionFailCount > 3:
                self.pauseMissionExecution(True)

    def connect(self):
        if (self.verbose):
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
        if (self.verbose):
            print("MissionPlanner: loadMission: loading mission")

        self.isMissionLoaded = False

        try:
            self.MISSIONS = self.missionLoader.load()
            self.isMissionLoaded = True
        except Exception as e:
            print("[ERR] MissionPlanner: loadMission exception:", str(e))

    def onMqttConnect(self, client, userdata, flags, rc):
        if self.verbose:
            print("MissionPlanner: onMqttConnect:", mqtt.connack_string(rc))

        self.mqttClient.subscribe([
            ("dji/status/connection", 1),
            ("dji/status/flight-mode", 1),
            ("dji/status/flight-control", 1)
            ("mission-planner/start", 1),
            ("mission-planner/pause", 1),
            ("mission-planner/shutdown", 1)
        ])

    def onMqttDisconnect(self, client, userdata, rc):
        if (self.verbose):
            print("MissionPlanner: onMqttDisconnect:", mqtt.connack_string(rc))

    def pauseMissionExecution(self, pause):
        if (self.verbose):
            print("MissionPlanner: pauseMissionExecution:", pause)

        self.isPauseMission = pause

    def onDJIConnectionStatus(self, client, userdata, msg):
        if (self.verbose):
            print("MissionPlanner: onDJIConnectionStatus:", msg.payload.decode())

        self.droneConnectionStatus = msg.payload.decode().lower() == "true"
        self.pauseMissionExecution(not self.droneConnectionStatus)

    def onDJIFlightModeStatus(self, client, userdata, msg):
        if (self.verbose):
            print("MissionPlanner: onDJIFlightModeStatus:", msg.payload.decode())

        self.droneFlightMode = msg.payload.decode()

    def onDJIFlightControlStatus(self, client, userdata, msg):
        if (self.verbose):
            print("MissionPlanner: onDJIFlightControlStatus:",
                  msg.payload.decode())

        self.droneFlightControlStatus = msg.payload.decode().lower() == "true"
        self.pauseMissionExecution(not self.droneFlightControlStatus)

    def onMissionPlannerStart(self, client, userdata, msg):
        if (self.verbose):
            print("MissionPlanner: onMissionPlannerStart:", msg.payload.decode())

        self.isStartMission = msg.payload.decode().lower() == "true"

    def onMissionPlannerPause(self, client, userdata, msg):
        if (self.verbose):
            print("MissionPlanner: onMissionPlannerPause:", msg.payload.decode())

        self.pauseMissionExecution(msg.payload.decode().lower() == "true")

    def onMissionPlannerShutdown(self, client, userdata, msg):
        if (self.verbose):
            print("MissionPlanner: onMissionPlannerShutdown:",
                  msg.payload.decode())

        if (msg.payload.decode().lower() == "true"):
            self.disconnect()
            quit()


parser = argparse.ArgumentParser()
parser.description = "Mission Planner for Panasonic DJI Drone"
parser.add_argument('--host', help="Hostname for MQTT broker",
                    type=str, required=True)
parser.add_argument(
    '-p', help="Port number for MQTT broker", dest="port", type=int, default=1883)
parser.add_argument(
    '-u', help="Username for MQTT authentication", dest="username", type=str, default="")
parser.add_argument(
    '-P', help="Password for MQTT authentication", dest="password", type=str, default=None)
parser.add_argument('-id', help="ID of the mission to be loaded",
                    dest="missionId", type=str, default="latest")
parser.add_argument('-f', help="The frequency for the execcutor to send control data (5 - 25)",
                    dest="freq", type=int, default=5)
parser.add_argument(
    '-v', help="Set the logging level to verbose", dest="verbose", action="store_true")
parser.set_defaults(verbose=False)
args = parser.parse_args()

if __name__ == "__main__":
    mp = MissionPlanner(args.host, args.port, args.username,
                        args.password, args.missionId, args.freq, args.verbose)
    mp.start()
