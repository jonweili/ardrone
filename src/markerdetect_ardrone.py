#!/usr/bin/env python
"""
    Detect markers using standard tag-detection on the ardrone

    Only for bottom camera!

"""

# Python
import time
import numpy as np
import math

# ROS
import rospy
from ardrone_autonomy.msg import Navdata
from ardrone.msg import Markers
from ardrone.msg import Marker


# Config
FREQUENCY = 0.5                     # max. frequency for updates
CAMERA_X_FACTOR = 1000.             # convert pixels to meter
CAMERA_Y_FACTOR = 1000.



# Init
UPDATE_DELAY = (1./FREQUENCY)
last_update_time = -1000

def callback_navdata(data):

    global last_update_time

    current_time = time.time()

    if (last_update_time + UPDATE_DELAY < current_time):

        markers = []

        # add all tags found
        # TODO: filter by type / add type to marker info

        # convert alt from mm to m
        height = data.altd / 1000.

        for i in range(data.tags_count):

            # center 500,500
            xc = data.tags_xc[i] - 500
            yc = data.tags_yc[i] - 500

            # convert from pixels to meters
            xc = xc * height / CAMERA_X_FACTOR
            yc = yc * height / CAMERA_Y_FACTOR

            # convert from camera to quadrotorframe
            x = -yc
            y = -xc

            # TODO: check for rotation
            theta = data.tags_orientation[i]

            marker = Marker()

            marker.x = x
            marker.y = y
            marker.theta = theta

            markers.append(marker)

        # publish detected markers

        pub_markers.publish(markers)
        last_update_time = current_time



def listener():

    rospy.Subscriber('/ardrone/navdata', Navdata, callback_navdata)

    rospy.spin() # keep running


if __name__ == '__main__':

    pub_markers = rospy.Publisher('markers', Markers)

    rospy.init_node('markerdetect_ardrone_tags', anonymous=True)

    rospy.loginfo("Starting marker detection (ARDrone tags)")

    listener()
