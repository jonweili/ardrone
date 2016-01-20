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
FREQUENCY = 4.                      # max. frequency for updates
CAMERA_X_FACTOR = 0.8                # convert pixels to meter. 1. = 90 degrees ( >1 = wider angle)
CAMERA_Y_FACTOR = 0.8                # measure at altitude 1m, visible width and height



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

        # TODO: for debug only

        if height < 1.:
            height = 1.

        for i in range(data.tags_count):

            # center 500,500 => -1, 1
            xc = (data.tags_xc[i] - 500.) / 500.
            yc = (data.tags_yc[i] - 500.) / 500.

            # convert from pixels to meters
            xc = xc * height * CAMERA_X_FACTOR
            yc = yc * height * CAMERA_Y_FACTOR

            # convert from camera to quadrotorframe
            x = -yc
            y = -xc

            # TODO: check for rotation
            theta = data.tags_orientation[i]

            marker = Marker()

            marker.x = x
            marker.y = y
            marker.theta = theta * 2. * math.pi / 360.

            markers.append(marker)
            print str(data.tags_xc[i]) + " / " + str(data.tags_yc[i]) + " / " + str(height)
            rospy.loginfo("tag detected at: " + str(round(marker.x,2)) + ", " + str(round(marker.y,2)) + " rad: " + str(round(marker.theta,3)))

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
