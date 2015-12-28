#!/usr/bin/env python
"""
    Detect markers in gazebo-world

"""

# Python
import time
import numpy as np
import math

# ROS
import rospy
from gazebo_msgs.msg import ModelStates
from ardrone.msg import Markers
from ardrone.msg import Marker


# Config
GAZEBO_QUADROTOR_NAME = "quadrotor" # name of quadrotor object
GAZEBO_MARKER_PREFIX = "marker"     # prefix for marker objects
FREQUENCY = 1.                      # (max) frequency for updates
CAMERA_ANGLE = 1.                   # 1 = 1:1 = 45 degrees, > 1 = eg. 2:1 = wide angle


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

        max_viewing_distance = quad_z / 2. * CAMERA_ANGLE

        quad_qw = data.pose[quadrotor_id].orientation.w
        quad_theta = math.acos(quad_qw) * 2

        markers = []

        for model_id in range(len(data.name)):
            if (data.name[model_id].startswith(GAZEBO_MARKER_PREFIX)):

                # convert world coordinates to local coordinates for quadrotor
                marker_x = data.pose[model_id].position.x
                marker_y = data.pose[model_id].position.y

                marker_qw = data.pose[model_id].orientation.w
                marker_theta = math.acos(marker_qw) * 2

                # only detect within viewing angle
                distance = math.sqrt((quad_x - marker_x)*(quad_x - marker_x) + (quad_y - marker_y)*(quad_y - marker_y))

                if distance < max_viewing_distance:

                    st = math.sin(quad_theta)
                    ct = math.cos(quad_theta)


                    # rotation matrix (in 2D)
                    R = np.array([[ct, -st],[st, ct]])

                    # rotationpoint = quadrotor (global)
                    A = np.array([[quad_x], [quad_y]])

                    B_global = np.array([[marker_x], [marker_y]])

                    B_local = np.dot( np.linalg.inv(R), (B_global - A))

                    theta_local = marker_theta - quad_theta


                    rospy.loginfo("tag detected at: " + str(B_local.item(0)) + ", " + str(B_local.item(1)) + " rad: " + str(theta_local))

                    marker = Marker()

                    marker.x = B_local.item(0)
                    marker.y = B_local.item(1)
                    marker.theta = theta_local

                    markers.append(marker)

        # publish detected markers

        pub_markers.publish(markers)



def listener():

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    rospy.spin() # keep running


if __name__ == '__main__':

    pub_markers = rospy.Publisher('markers', Markers)

    rospy.init_node('markerdetect_gazebo_simulator', anonymous=True)

    rospy.loginfo("Starting simulaten marker detection (Gazebo)")

    listener()
