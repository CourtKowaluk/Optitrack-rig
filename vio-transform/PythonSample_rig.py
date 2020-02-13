# OptiTrack NatNet direct depacketization sample for Python 3.x
#
# Uses the Python NatNetClient.py library to establish a connection (by creating a NatNetClient),
# and receive data via a NatNet connection and decode it using the NatNetClient library.
# Receives data that is intercepted by udp_relay.c, and republishes rigid bodies to an MQTT Topic. 
# Uses the Paho Python and MQTT libraries. 


import paho.mqtt.client as mqtt
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation

from NatNetClient import NatNetClient


#The broker and port to connect to send MQTT messages.
broker_url = "oz.andrew.cmu.edu"
broker_port = 1883

name = "camera_pixel_pixel"
optitrack_id = 5

viotopic = "/topic/vio/" + name
rigtopic = "realm/s/earth/" + name

# Object pose according to two systems
last_optitrack_time = 0
last_optitrack_pose = np.identity(4)
MAX_TIME_DELTA = 100000 # 100ms


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame(id, position, rotation):
    if id == optitrack_id: #if the rigid body id (in Motive settings) is set to 5, so we don't get other rigid bodies we don't want
        last_optitrack_pose[0:2, 0:2] = Rotation.from_quat(rotation).as_matrix()
        last_optitrack_pose[0:2, 3] = position
        last_optitrack_time = time.time()


def on_connect(client, userdata, flags, rc):
    print("Connected With Result Code "+rc)


def on_message(client, userdata, message):
    msg = message.payload.decode()
    js = json.loads(msg)

    rot = js["rotation"]
    rx = rot["x"]
    ry = rot["y"]
    rz = rot["z"]
    rw = rot["w"]

    pos = js["position"]
    px = pos["x"]
    py = pos["y"]
    pz = pos["z"]

    vio_pose = np.identity(4)
    vio_pose[0:2, 0:2] = Rotation.from_quat([x, y, z, w]).as_matrix()
    vio_pose[0:2, 3] = [px, py, pz]

    rig_pose = last_optitrack_pose.dot(np.linalg.inv(vio_pose))
    rig_quat = Rotation.from_matrix(rig_pose[0:2, 0:2]).as_quat()
    rig_pos = rig_pose[0:2, 3]

    rigmessage = {
        "object_id": name,
        "action": "update",
        "type": "rig",
        "data": {
            "position": {
                "x": rig_pos[0, 3],
                "y": rig_pos[1, 3],
                "z": rig_pos[2, 3]
            },
            "rotation": {
                "x": rig_quat[0],
                "y": rig_quat[1],
                "z": rig_quat[2],
                "w": rig_quat[3]
            }
        }
    }

    # TODO should probably use timestamps inside message?
    # TODO should probably only send if rig changes a lot?
    tdiff = time.time() - last_optitrack_time
    if tdiff > 0 and tdiff < MAX_TIME_DELTA:
        client.publish(rigtopic, json.dumps(rigmessage), qos=1, retain=False)
    else:
        print("optitrack too old (%f ms)", tdiff * 1000.0)


# This will create a new NatNet client
streamingClient = NatNetClient()

#connect to the MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker_url, broker_port)
client.subscribe(viotopic, qos=1)

# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streamingClient.rigidBodyListener = receiveRigidBodyFrame

# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
streamingClient.run()

#loop the mqtt client so it doesn't disconnect
client.loop_forever()

