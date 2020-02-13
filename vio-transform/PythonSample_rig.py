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
broker_url = 'oz.andrew.cmu.edu'
broker_port = 1883

name = 'camera_pixel_pixel'
optitrack_id = 5

viotopic = '/topic/vio/' + name
rigtopic = 'realm/s/earth/' + name

# Object pose according to two systems
last_optitrack_time = 0
last_optitrack_pose = np.identity(4)
MAX_TIME_DELTA = 0.1 # 100ms

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame(id, position, rotation):
    global last_optitrack_time
    global last_optitrack_pose
  
    if(id==2): 
        message = {
        "object_id": "cube_t",
        "action": "update",
        "type": "object",
        "data": {
           "position": {
                "x": position[0],
                "y": position[1],
                "z": position[2]
            },
            "rotation": {
                "x": rotation[0],
                "y": rotation[1],
                "z": rotation[2],
                "w": rotation[3]
            },}}
        client.publish("realm/s/earth/cube_t", json.dumps(message), qos=1, retain=False)
        #print(json.dumps(message))

    if id == optitrack_id: #if the rigid body id (in Motive settings) is set to 5, so we don't get other rigid bodies we don't want
        new_optitrack_pose = np.identity(4)
        new_optitrack_pose[0:3, 0:3] = Rotation.from_quat(rotation).as_matrix()
        new_optitrack_pose[0:3, 3] = position
        if (new_optitrack_pose != last_optitrack_pose).any():
            last_optitrack_time = time.time()
            last_optitrack_pose = new_optitrack_pose


def on_connect(client, userdata, flags, rc):
    print('Connected With Result Code ' + rc)


def on_message(client, userdata, message):
    global last_optitrack_time
    global last_optitrack_pose
    msg = message.payload.decode()
    
    js = json.loads(msg)
    
    rot = js['data']['rotation']
    rx = rot['x']
    ry = rot['y']
    rz = rot['z']
    rw = rot['w']

    pos = js['data']['position']
    px = pos['x']
    py = pos['y']
    pz = pos['z']

    vio_pose = np.identity(4)
    vio_pose[0:3, 0:3] = Rotation.from_quat([rx, ry, rz, rw]).as_matrix()
    vio_pose[0:3, 3] = [px, py, pz]
    #print("vio pose: ")
    #print(vio_pose)

    rig_pose = last_optitrack_pose.dot(np.linalg.inv(vio_pose))
    #print("rig pose: ")
    #print(rig_pose)
    rig_quat = Rotation.from_matrix(rig_pose[0:3, 0:3]).as_quat()
    rig_pos = rig_pose[0:3, 3]

    rigmessage = {
        'object_id': name,
        'action': 'update',
        'type': 'rig',
        'data': {
            'position': {
                'x': rig_pos[0],
                'y': rig_pos[1],
                'z': rig_pos[2]
            },
            'rotation': {
                'x': rig_quat[0],
                'y': rig_quat[1],
                'z': rig_quat[2],
                'w': rig_quat[3]
            }
        }
    }

    # TODO should probably use timestamps inside message?
    # TODO should probably only send if rig changes a lot?
    tdiff = time.time() - last_optitrack_time
    if tdiff > 0 and tdiff < MAX_TIME_DELTA:
        client.publish(rigtopic, json.dumps(rigmessage), qos=1, retain=False)
        print(tdiff)
    else:
        print('optitrack too old (%lf s)' % (tdiff))


# This will create a new NatNet client
streamingClient = NatNetClient()

#connect to the MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker_url, broker_port)
client.subscribe(viotopic, qos=1)
print(viotopic)

# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streamingClient.newFrameListener = None
streamingClient.rigidBodyListener = receiveRigidBodyFrame

# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
streamingClient.run()

#loop the mqtt client so it doesn't disconnect
client.loop_forever()

