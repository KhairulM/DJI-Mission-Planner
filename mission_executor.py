import time
import re
import paho.mqtt.client as mqtt
from mission import Mission
from mission_type import MissionType

ALTITUDE_ERR = 0.3


class MissionExecutor:
    def __init__(self, username, password, freq, verbose):
        self.freq = freq
        self.verbose = verbose
        self.isMqttConnected = False

        self.mqttClient = mqtt.Client()
        self.mqttClient.max_inflight_messages_set(50)
        self.mqttClient.username_pw_set(username, password)

        self.mqttClient.message_callback_add("cv/status", self.onCvStatus)
        self.mqttClient.message_callback_add("cv/barcode", self.onCvBarcode)
        self.mqttClient.message_callback_add(
            "dji/control/takeoff/result", self.onDJITakeoffResult)
        self.mqttClient.message_callback_add(
            "dji/control/rth/result", self.onDJIRTHResult)
        self.mqttClient.message_callback_add(
            "dji/status/altitude", self.onDJIAltitude)

        self.droneAltitude = 0.0
        self.droneTakeoffResult = None
        self.droneRthResult = None
        self.barcodePos = []

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
    def execute(self, mission: Mission, maxAlt, missionSpeed):
        if (self.verbose):
            print("MissionExecutor: Executing mission:",
                  mission.typeString, str(mission.argument))

        try:
            if (mission.type == MissionType.up_to or
                    mission.type == MissionType.down_to):
                return self.moveZ(abs(mission.argument[0]), maxAlt, 0.5)

            if (mission.type == MissionType.up):
                return self.moveZ(self.droneAltitude + abs(mission.argument[0]), maxAlt, 0.5)

            if (mission.type == MissionType.down):
                return self.moveZ(self.droneAltitude - abs(mission.argument[0]), maxAlt, 0.5)

            if (mission.type == MissionType.right):
                return self.moveX(abs(mission.argument[0]), abs(missionSpeed))

            if (mission.type == MissionType.left):
                return self.moveX(abs(mission.argument[0]), -abs(missionSpeed))

            if (mission.type == MissionType.rotate):
                return self.rotate(mission.argument[0])

            if (mission.type == MissionType.takeoff):
                return self.sendTakeoff()

            if (mission.type == MissionType.rth):
                return self.sendRTH()

            if (mission.type == MissionType.align_with_barcode):
                return self.alignWithBarcode(mission)

        except Exception as e:
            print("[ERR] MissionExecutor: execute exception:", str(e))
            return -1

    def moveX(self, distance, missionSpeed):
        controlData = [missionSpeed, 0.0, 0.0, 0.0]
        second = distance/abs(missionSpeed)

        res = 0
        start = time.time()

        while time.time() - start < second:
            res |= self.sendControlData(controlData)

        return res

    def moveY(self, distance, missionSpeed):
        controlData = [0.0, missionSpeed, 0.0, 0.0]
        second = distance/abs(missionSpeed)

        res = 0
        start = time.time()

        while time.time() - start < second:
            res |= self.sendControlData(controlData)

        return res

    def moveZ(self, alt, maxAlt, minAlt):
        alt = min(max(alt, minAlt), maxAlt)

        res = self.sendControlData([0.0, 0.0, 0.0, alt])

        if (res != 0):
            return res

        while abs(self.droneAltitude - alt) > ALTITUDE_ERR:
            continue

        return res

    def rotate(self, degree):
        rotationSpeed = 5.0
        controlData = [0.0, 0.0, rotationSpeed if degree >
                       0 else -rotationSpeed, 0.0]

        second = abs(degree)/rotationSpeed
        res = 0
        start = time.time()

        while time.time() - start < second:
            res |= self.sendControlData(controlData)

        return res

    def sendControlData(self, controlData):
        if (not self.mqttClient.is_connected() or len(controlData) != 4):
            return -1

        if (self.verbose):
            print("MissionExecutor: Sending control data:", controlData)

        for i in range(self.freq):
            self.mqttClient.publish("dji/control", str(controlData), 2)
            time.sleep(1.0/self.freq)

        return 0

    def sendTakeoff(self):
        if (not self.mqttClient.is_connected()):
            return -1

        if (self.verbose):
            print("MissionExecutor: Sending takeoff comand")

        self.droneTakeoffResult = None
        self.mqttClient.publish("dji/control/takeoff", "true", 2)

        while self.droneTakeoffResult == None or self.droneTakeoffResult == "started":
            continue

        if self.droneTakeoffResult == "failed":
            return -1
        elif self.droneTakeoffResult == "completed":
            return 0

    def sendRTH(self):
        if (not self.mqttClient.is_connected()):
            return -1

        if (self.verbose):
            print("MissionExecutor: Sending takeoff comand")

        self.droneRthResult = None
        self.mqttClient.publish("dji/control/rth", "true", 2)

        while self.droneRthResult == None or self.droneRthResult == "started":
            continue

        if self.droneRthResult == "failed":
            return -1
        elif self.droneRthResult == "completed":
            return 0

    def alignWithBarcode(self, mission):
        if (not self.mqttClient.is_connected()):
            return -1

        return 0

    def onMqttConnect(self, client, userdata, flags, rc):
        if self.verbose:
            print("MissionExecutor: onMqttConnect:", mqtt.connack_string(rc))

        self.isMqttConnected = rc == 0

        self.mqttClient.subscribe([
            ("cv/status", 2),
            ("cv/barcode", 2),
            ("dji/status/altitude", 2),
            ("dji/control/takeoff/result", 2),
            ("dji/control/rth/result", 2)
        ])

    def onMqttDisconnect(self, client, userdata, rc):
        if self.verbose:
            print("MissionExecutor: onMqttConnect:", mqtt.connack_string(rc))

        self.isMqttConnected = False

    def onCvStatus(self, client, userdata, msg):
        if (self.verbose):
            print("MissionExecutor: onCvStatus:", msg.payload.decode())

    def onCvBarcode(self, client, userdata, msg):
        if self.verbose:
            print("MissionExecutor: onCvBarcode:", msg.payload.decode)

        self.barcodePos = self.parseCvBarcode(msg.payload.decode)

    def parseCvBarcode(msgString):
        pos = re.findall('[0-9]+', msgString)

        barcodePos = []
        for i in range(0, len(pos), 2):
            barcodePos.append((pos[i], pos[i+1]))

    def onDJITakeoffResult(self, client, userdata, msg):
        if (self.verbose):
            print("MissionExecutor: onDJITakeoffResult:", msg.payload.decode())

        self.droneTakeoffResult = msg.payload.decode().lower()

    def onDJIRTHResult(self, client, userdata, msg):
        if (self.verbose):
            print("MissionExecutor: onDJIRTHResult:", msg.payload.decode())

        self.droneRthResult = msg.payload.decode().lower()

    def onDJIAltitude(self, client, userdata, msg):
        if (self.verbose):
            print("MissionExecutor: onDJIAltitude:", msg.payload.decode())

        self.droneAltitude = float(msg.payload.decode())
