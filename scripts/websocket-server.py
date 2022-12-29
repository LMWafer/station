#! /usr/bin/python3

import json

import numpy as np
import rospy
import tf
from fastapi import FastAPI, WebSocket
import pytransform3d.rotations as ptr

rospy.init_node("websocket", anonymous=True)
rospy.loginfo("Initializing node with name 'websocket'.")
listener = tf.TransformListener()

parameters = rospy.get_param("/robotlab/world")
objects_shape = parameters["objects"]["shapes"]
expected_objects_ids = [int(id) for id in objects_shape.keys()]

app = FastAPI()

@app.websocket("/ros")
async def endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        message = await websocket.receive_json()
        # print(message)
        if message["key"] == "world_state":
            # print("Sending state")
            # now = rospy.Time.now()
            try:
                state = {"positions": [], "scales": [], "rotations": []}
                for id in expected_objects_ids:
                    # listener.waitForTransform("/object_1", "/object_2", now, rospy.Duration(4.0))
                    tvec, qvec = listener.lookupTransform(f"/object_{id}", "/object_1", rospy.Time(0))
                    rvec = ptr.extrinsic_euler_xyz_from_active_matrix(ptr.matrix_from_quaternion(np.roll(qvec, 1))).tolist()
                    state["positions"].extend(tvec)
                    state["scales"].extend([1, 1, 1])
                    state["rotations"].extend(rvec)
                await websocket.send_json(state)
                    # await websocket.close()
            except Exception as e:
                rospy.logerr(f"Error during retrieval of frame")
                print(e)

        elif message["key"] == "world_config":
            # print("Sending parameters")
            try:
                parameters = rospy.get_param("/robotlab/world")
                await websocket.send_json(parameters)
            except Exception as e:
                rospy.logerr(f"Error during retrieval of parameters")

