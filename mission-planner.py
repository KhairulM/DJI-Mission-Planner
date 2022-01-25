import argparse
import requests
import paho.mqtt.client as mqtt

parser = argparse.ArgumentParser()
parser.description = "Mission Planner for Panasonic DJI Drone"
parser.add_argument('--host', help="Hostname for MQTT broker",
                    type=str, required=True)
parser.add_argument(
    '-p', help="Port number for MQTT broker", type=int, default=1883)
parser.add_argument(
    '-u', help="Username for MQTT authentication", type=str, default="")
parser.add_argument(
    '-P', help="Password for MQTT authentication", type=str, default=None)
parser.add_argument(
    '-v', help="Set the logging level to verbose", dest="verbose", action="store_true")
parser.set_defaults(verbose=False)
args = parser.parse_args()

mHost = args.host
mPort = args.p
mUsername = args.u
mPassword = args.P
mClient = mqtt.Client()

mDroneConnectionStatus = False
mDroneFlightMode = ""


def pauseMission():
    # TODO: IMPLEMENT PAUSING MISSION
    pass


def onDJIConnectionStatus(client, userdata, msg):
    if (args.verbose):
        print("onDJIConnectionStatus:", msg.payload.decode())

    mDroneConnectionStatus = msg.payload.decode().lower() == 'true'

    if (not mDroneConnectionStatus):
        pauseMission()


def onDJIFlightModeStatus(client, userdata, msg):
    if (args.verbose):
        print("onDJIFlightModeStatus:", msg.payload.decode())

    mDroneFlightMode = msg.payload.decode()


def onMissionPlannerShutdown(client, userdata, msg):
    if (args.verbose):
        print("onMissionPlannerShutdown:", msg.payload.decode())

    # TODO: IMPLEMENT GRACEFUL SHUTDOWN
    quit()


def onCvStatus(client, userdata, msg):
    if (args.verbose):
        print("onCvStatus:", msg.payload.decode())


def onMqttConnect(client, userdata, flags, rc):
    if args.verbose:
        print("onMqttConnect:", rc, flags)

    client.subscribe([
        ("dji/status/connection", 1),
        ("dji/status/flight-mode", 1),
        ("mission-planner/shutdown", 1),
        ("cv/status", 2)
    ])


def init():
    mClient.max_inflight_messages_set(50)
    mClient.username_pw_set(mUsername, mPassword)

    mClient.message_callback_add(
        "dji/status/connection", onDJIConnectionStatus)
    mClient.message_callback_add(
        "dji/status/flight-mode", onDJIFlightModeStatus)
    mClient.message_callback_add(
        "mission-planner/shutdown", onMissionPlannerShutdown)
    mClient.message_callback_add("cv/status", onCvStatus)


def connect():
    mClient.on_connect = onMqttConnect
    mClient.connect(mHost, mPort)


if __name__ == "__main__":
    init()
    connect()
    mClient.loop_forever()
