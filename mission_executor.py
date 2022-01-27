import time
import paho.mqtt.client as mqtt
from mission import Mission
from mission_type import MissionType


class MissionExecutor:
    def __init__(self, username, password, freq, verbose):
        self.freq = freq
        self.verbose = verbose
        self.isMqttConnected = False

        self.mqttClient = mqtt.Client()
        self.mqttClient.max_inflight_messages_set(50)
        self.mqttClient.username_pw_set(username, password)

        self.mqttClient.message_callback_add("cv/status", self.onCvStatus)
        self.mqttClient.message_callback_add(
            "dji/status/altitude", self.onDJIAltitude)

        self.droneAltitude = None

    def connect(self, host, port):
        self.mqttClient.on_connect = self.onMqttConnect
        self.mqttClient.connect(host, port)
        self.mqttClient.loop_start()

    def disconnect(self):
        self.mqttClient.on_disconnect = self.onMqttDisconnect
        self.mqttClient.disconnect()
        self.mqttClient.loop_stop()

    # Sending control messages to the drone
    # Return: Error code for mission execution
    # 0: success
    # -1: failed
    def execute(self, mission: Mission):
        if (self.verbose):
            print("MissionExecutor: Executing mission:",
                  mission.typeString, str(mission.argument))

        try:
            if (mission.type == MissionType.up or
                mission.type == MissionType.down or
                mission.type == MissionType.right or
                mission.type == MissionType.left or
                    mission.type == MissionType.rotate):
                return self.sendControlData(mission.controlData)

            if (mission.type == MissionType.takeoff):
                self.sendTakeoff()

            if (mission.type == MissionType.rth):
                self.sendRTH()

        except Exception as e:
            print("[ERR] MissionExecutor: execute exception:", str(e))
            return -1

    def sendControlData(self, controlData):
        if (not self.mqttClient.is_connected()):
            return -1

        if (self.verbose):
            print("MissionExecutor: Sending control data:", controlData)

        for i in range(self.freq):
            self.mqttClient.publish("dji/control", str(controlData))
            time.sleep(1.0/self.freq)

        return 0

    def sendTakeoff(self):
        if (not self.mqttClient.is_connected()):
            return -1

        if (self.verbose):
            print("MissionExecutor: Sending takeoff comand")

        self.mqttClient.publish("dji/control/takeoff", "true", 2)

        return 0

    def sendRTH(self):
        if (not self.mqttClient.is_connected()):
            return -1

        if (self.verbose):
            print("MissionExecutor: Sending takeoff comand")

        self.mqttClient.publish("dji/control/rth", "true", 2)

        return 0

    def onMqttConnect(self, client, userdata, flags, rc):
        if self.verbose:
            print("MissionExecutor: onMqttConnect:", mqtt.connack_string(rc))

        self.isMqttConnected = rc == 0

        self.mqttClient.subscribe([
            ("cv/status", 2),
            ("dji/status/altitude", 2)
        ])

    def onMqttDisconnect(self, client, userdata, rc):
        if self.verbose:
            print("MissionExecutor: onMqttConnect:", mqtt.connack_string(rc))

        self.isMqttConnected = False

    def onCvStatus(self, client, userdata, msg):
        if (self.verbose):
            print("MissionExecutor: onCvStatus:", msg.payload.decode())

    def onDJIAltitude(self, client, userdata, msg):
        if (self.verbose):
            print("MissionExecutor: onDJIAltitude:", msg.payload.decode())

        self.droneAltitude = float(msg.payload.decode())
