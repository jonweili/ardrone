#!/usr/bin/env python
"""
    Publish drone state, dummy: gets data directly from Gazebo

"""

# Python
import time
import numpy as np
import math

# ROS
import rospy
from gazebo_msgs.msg import ModelStates
from ardrone.msg import DroneState

# Config
GAZEBO_QUADROTOR_NAME = "quadrotor" # name of quadrotor object
FREQUENCY = 10.                     # max. frequency for updates

# Init
UPDATE_DELAY = (1./FREQUENCY)
last_update_time = -1000

def callback(data):
    global last_update_time

    current_time = time.time()

    if (last_update_time + UPDATE_DELAY < current_time):

        last_update_time = current_time

        # find quadrotor in gazebo
        quadrotor_id = -1
        for model_id in range(len(data.name)):
            if (data.name[model_id] == GAZEBO_QUADROTOR_NAME):
                # quadrotor found, keep id
                quadrotor_id = model_id
                #print "found quadrotor: " + str(quadrotor_id)

        # Stop when no quadrotor found
        assert quadrotor_id > -1, "No quadrotor named '" + GAZEBO_QUADROTOR_NAME + "' found in Gazebo world"

        quad_x = data.pose[quadrotor_id].position.x
        quad_y = data.pose[quadrotor_id].position.y
        quad_z = data.pose[quadrotor_id].position.z

        quad_qw = data.pose[quadrotor_id].orientation.w
        quad_theta = math.acos(quad_qw) * 2

        droneState = DroneState()

        droneState.x = quad_x
        droneState.y = quad_y
        droneState.z = quad_z
        droneState.theta = quad_theta

        # publish drone state

        rospy.loginfo("drone at: " + str(droneState.x) + ", " + str(droneState.y) + ", " + str(droneState.z)+ " rad: " + str(droneState.theta))


        pub_markers.publish(droneState)


def listener():

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    rospy.spin() # keep running


if __name__ == '__main__':

    pub_markers = rospy.Publisher('dronestate', DroneState)

    rospy.init_node('dronestate_gazebo_simulator', anonymous=True)

    rospy.loginfo("Starting simulated dronestate (Gazebo)")

    listener()
