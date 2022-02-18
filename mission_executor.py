import time
import json
import paho.mqtt.client as mqtt
from mission import Mission
from mission_type import MissionType

TOPIC_CV_STATUS = "cv/status"
TOPIC_CV_ARUCO_POSITION = "cv/aruco-position"
TOPIC_CV_RUN_DETECTION = "cv/run-detection"
TOPIC_DJI_GIMBAL = "dji/gimbal"
TOPIC_DJI_GIMBAL_RESET = "dji/gimbal/reset"
TOPIC_DJI_CONTROL = "dji/control"
TOPIC_DJI_CONTROL_TAKEOFF = "dji/control/takeoff"
TOPIC_DJI_CONTROL_RTH = "dji/control/rth"
TOPIC_DJI_CONTROL_LAND = "dji/control/land"
TOPIC_DJI_CONTROL_TAKEOFF_RESULT = "dji/control/takeoff/result"
TOPIC_DJI_CONTROL_RTH_RESULT = "dji/control/rth/result"
TOPIC_DJI_CONTROL_LAND_RESULT = "dji/control/land/result"
TOPIC_DJI_STATUS_ALTITUDE = "dji/status/altitude"
TOPIC_MISSION_PLANNER_RACK_ID = "mission-planner/rack-id"
TOPIC_MISSION_PLANNER_START_RESULT = "mission-planner/start/result"

ALTITUDE_ERR = 0.12
BOUNDARY_Y_HIGH = 720
BOUNDARY_Y_LOW = 576
BOUNDARY_X_HIGH = 576
BOUNDARY_X_LOW = 384


class MissionExecutor:
    def __init__(self, username, password, freq, verbose):
        self.freq = freq
        self.verbose = verbose

        self.mqttClient = mqtt.Client()
        self.mqttClient.max_inflight_messages_set(50)
        self.mqttClient.username_pw_set(username, password)

        self.mqttClient.message_callback_add(TOPIC_CV_STATUS, self.onCvStatus)
        self.mqttClient.message_callback_add(
            TOPIC_CV_ARUCO_POSITION, self.onCvArucoPosition
        )
        self.mqttClient.message_callback_add(
            TOPIC_DJI_CONTROL_TAKEOFF_RESULT, self.onDJITakeoffResult
        )
        self.mqttClient.message_callback_add(
            TOPIC_DJI_CONTROL_RTH_RESULT, self.onDJIRTHResult
        )
        self.mqttClient.message_callback_add(
            TOPIC_DJI_STATUS_ALTITUDE, self.onDJIAltitude
        )
        self.mqttClient.message_callback_add(
            TOPIC_DJI_CONTROL_LAND_RESULT, self.onDJILandResult
        )

        self.droneAltitude = 0.0
        self.droneTakeoffResult = None
        self.droneRthResult = None
        self.droneLandResult = None
        self.arucoPos = dict()
        self.cvStatus = None

        self.maxAlt = 0
        self.minAlt = 0
        self.missionSpeed = 0

    def connect(self, host, port):
        self.mqttClient.on_connect = self.onMqttConnect
        self.mqttClient.connect(host, port)
        self.mqttClient.loop_start()

    def disconnect(self):
        self.mqttClient.on_disconnect = self.onMqttDisconnect
        self.mqttClient.disconnect()
        self.mqttClient.loop_stop()

    def setMaxAltitude(self, maxAlt):
        self.maxAlt = maxAlt

    def setMinAltitude(self, minAlt):
        self.minAlt = minAlt

    def setMissionSpeed(self, speed):
        self.missionSpeed = speed

    # Sending control messages to the drone
    # Return: Error code for mission execution
    # 0: success
    # -1: failed
    def execute(self, mission: Mission):
        if self.verbose:
            print(
                "MissionExecutor: Executing mission:",
                mission.typeString,
                str(mission.argument),
            )

        self.mqttClient.publish(
            TOPIC_MISSION_PLANNER_START_RESULT,
            "%s %s" % (mission.typeString, str(mission.argument)),
            2,
            True,
        )

        try:
            if mission.type == MissionType.wait_for_alt:
                return self.moveZ(abs(mission.argument[0]))

            if mission.type == MissionType.up:
                return self.moveZ(self.droneAltitude + abs(mission.argument[0]))

            if mission.type == MissionType.down:
                return self.moveZ(self.droneAltitude - abs(mission.argument[0]))

            if mission.type == MissionType.right:
                return self.moveX(abs(mission.argument[0]), abs(self.missionSpeed))

            if mission.type == MissionType.left:
                return self.moveX(abs(mission.argument[0]), -abs(self.missionSpeed))

            if mission.type == MissionType.rotate:
                return self.rotate(mission.argument[0])

            if mission.type == MissionType.takeoff:
                return self.sendTakeoff()

            if mission.type == MissionType.rth:
                return self.sendRTH()

            if mission.type == MissionType.land:
                return self.sendLand()

            if mission.type == MissionType.align_with_barcode:
                return self.alignWithBarcode(mission.argument[0])

            if mission.type == MissionType.wait_for_cv:
                return self.waitForCv(mission.argument[0])

            if mission.type == MissionType.publish_rack_id:
                return self.publishRackId(mission.argument[0])

        except Exception as e:
            print("[ERR] MissionExecutor: execute exception:", str(e))
            return -1

    def moveX(self, distance, speed):
        if speed == 0 or distance == 0:
            return 0

        controlData = [speed, 0.0, 0.0, self.droneAltitude]
        second = distance / abs(speed)

        res = 0
        start = time.time()

        while time.time() - start < second:
            res |= self.sendControlData(controlData)

        return res

    def moveY(self, distance, speed):
        if speed == 0 or distance == 0:
            return 0

        controlData = [0.0, speed, 0.0, self.droneAltitude]
        second = distance / abs(speed)

        res = 0
        start = time.time()

        while time.time() - start < second:
            res |= self.sendControlData(controlData)

        return res

    def moveZ(self, alt):
        alt = min(max(alt, self.minAlt), self.maxAlt)

        failCount = 0

        while failCount < 3 and abs(self.droneAltitude - alt) > ALTITUDE_ERR:
            res = self.sendControlData([0.0, 0.0, 0.0, alt])

            if res == -1:
                failCount += 1

        if failCount >= 3:
            return -1

        return 0

    def rotate(self, degree):
        rotationSpeed = 45.0
        controlData = [
            0.0,
            0.0,
            rotationSpeed if degree > 0 else -rotationSpeed,
            self.droneAltitude,
        ]

        second = abs(degree) / rotationSpeed
        res = 0
        start = time.time()

        while time.time() - start < second:
            res |= self.sendControlData(controlData)

        return res

    def sendControlData(self, controlData):
        if not self.mqttClient.is_connected() or len(controlData) != 4:
            return -1

        if self.verbose:
            print("MissionExecutor: Sending control data:", controlData)

        self.mqttClient.publish(
            TOPIC_MISSION_PLANNER_START_RESULT,
            "sending control data: " + str(controlData),
            2,
            True,
        )

        self.mqttClient.publish(TOPIC_DJI_CONTROL, str(controlData), 2)

        time.sleep(1 / self.freq)

        return 0

    def sendTakeoff(self):
        if not self.mqttClient.is_connected():
            return -1

        if self.verbose:
            print("MissionExecutor: Sending takeoff comand")

        self.droneTakeoffResult = None
        self.mqttClient.publish(TOPIC_DJI_CONTROL_TAKEOFF, "true", 2)

        while self.droneTakeoffResult == None or self.droneTakeoffResult == "started":
            continue

        if self.droneTakeoffResult == "failed":
            return -1
        elif self.droneTakeoffResult == "completed":
            return 0

    def sendRTH(self):
        if not self.mqttClient.is_connected():
            return -1

        if self.verbose:
            print("MissionExecutor: Sending takeoff comand")

        self.droneRthResult = None
        self.mqttClient.publish(TOPIC_DJI_CONTROL_RTH, "true", 2)

        while self.droneRthResult == None or self.droneRthResult == "started":
            continue

        if self.droneRthResult == "failed":
            return -1
        elif self.droneRthResult == "completed":
            return 0

    def sendLand(self):
        if not self.mqttClient.is_connected():
            return -1

        if self.verbose:
            print("MissionExecutor: Sending land comand")

        self.droneLandResult = None
        self.mqttClient.publish(TOPIC_DJI_CONTROL_LAND, "true", 2)

        while self.droneLandResult == None or self.droneLandResult == "started":
            continue

        if self.droneLandResult == "failed":
            return -1
        elif self.droneLandResult == "completed":
            return 0

    def alignWithBarcode(self, rackId):
        if not self.mqttClient.is_connected():
            return -1

        if self.verbose:
            print("MissionExecutor: Aligning with barcode")

        # TODO: SMARTER IMPLEMENTATION PLS, SMH
        isNotAligned = True
        failCount = 0

        while isNotAligned:
            if failCount >= 3:
                return -1

            if rackId not in self.arucoPos:
                failCount += 1
                continue

            barcodePos = self.arucoPos[rackId]
            barcodePos = [
                (barcodePos[0][0] + barcodePos[1][0]) / 2,
                (barcodePos[0][1] + barcodePos[1][1]) / 2,
            ]

            res = 0
            isNotAligned = False

            if barcodePos[0] >= BOUNDARY_X_HIGH:
                isNotAligned = True
                res |= self.moveX(0.1, -0.1)
            if barcodePos[0] <= BOUNDARY_X_LOW:
                isNotAligned = True
                res |= self.moveX(0.1, 0.1)
            if barcodePos[1] >= BOUNDARY_Y_HIGH:
                isNotAligned = True
                res |= self.sendControlData([0.0, 0.0, 0.0, self.droneAltitude - 0.1])
            if barcodePos[1] <= BOUNDARY_Y_LOW:
                isNotAligned = True
                res |= self.sendControlData([0.0, 0.0, 0.0, self.droneAltitude + 0.1])

            if res == -1:
                failCount += 1

            self.arucoPos = dict()

        return 0

    def waitForCv(self, isHighestLevel):
        if not self.mqttClient.is_connected():
            return -1

        if self.verbose:
            print("MissionExecutor: waitForCv: waiting for cv status to > 0")

        self.mqttClient.publish(TOPIC_CV_RUN_DETECTION, "true", 1, True)

        maxGimbalPitchAngle = 70.0

        maxtime = 10.0
        startTime = time.time()
        endTime = time.time()

        while self.cvStatus == None or (
            self.cvStatus["status"] == 1 and endTime - startTime < maxtime
        ):
            if isHighestLevel:
                self.mqttClient.publish(
                    TOPIC_DJI_GIMBAL,
                    (endTime - startTime) / maxtime * maxGimbalPitchAngle,
                    1,
                )

            if self.cvStatus["status"] == 1:
                self.mqttClient.publish(TOPIC_CV_RUN_DETECTION, "true", 1)

            endTime = time.time()
            continue

        self.mqttClient.publish(TOPIC_CV_RUN_DETECTION, "false", 1)

        self.mqttClient.publish(TOPIC_DJI_GIMBAL, 0, 1)

        if endTime - startTime >= maxtime:
            return -1

        return 0

    def publishRackId(self, rackId):
        if not self.mqttClient.is_connected():
            return -1

        if self.verbose:
            print("MissionExecutor: Aligning with barcode")

        self.mqttClient.publish(TOPIC_MISSION_PLANNER_RACK_ID, rackId, 2, True)

        return 0

    def onMqttConnect(self, client, userdata, flags, rc):
        if self.verbose:
            print("MissionExecutor: onMqttConnect:", mqtt.connack_string(rc))

        self.mqttClient.subscribe(
            [
                (TOPIC_CV_STATUS, 2),
                (TOPIC_CV_ARUCO_POSITION, 2),
                (TOPIC_DJI_STATUS_ALTITUDE, 2),
                (TOPIC_DJI_CONTROL_TAKEOFF_RESULT, 2),
                (TOPIC_DJI_CONTROL_RTH_RESULT, 2),
                (TOPIC_DJI_CONTROL_LAND_RESULT, 2),
            ]
        )

    def onMqttDisconnect(self, client, userdata, rc):
        if self.verbose:
            print("MissionExecutor: onMqttConnect:", mqtt.connack_string(rc))

    def onCvStatus(self, client, userdata, msg):
        if self.verbose:
            print("MissionExecutor: onCvStatus:", msg.payload.decode())

        self.cvStatus = json.loads(msg.payload.decode())

    def onCvArucoPosition(self, client, userdata, msg):
        if self.verbose:
            print("MissionExecutor: onCvArucoPosition:", msg.payload.decode())

        self.arucoPos = json.loads(msg.payload.decode())

    def onDJITakeoffResult(self, client, userdata, msg):
        if self.verbose:
            print("MissionExecutor: onDJITakeoffResult:", msg.payload.decode())

        self.droneTakeoffResult = msg.payload.decode().lower()

    def onDJIRTHResult(self, client, userdata, msg):
        if self.verbose:
            print("MissionExecutor: onDJIRTHResult:", msg.payload.decode())

        self.droneRthResult = msg.payload.decode().lower()

    def onDJILandResult(self, client, userdata, msg):
        if self.verbose:
            print("MissionExecutor: onDJILandResult:", msg.payload.decode())

        self.droneLandResult = msg.payload.decode().lower()

    def onDJIAltitude(self, client, userdata, msg):
        # if (self.verbose):
        #     print("MissionExecutor: onDJIAltitude:", msg.payload.decode())

        self.droneAltitude = float(msg.payload.decode())
