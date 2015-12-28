#!/usr/bin/env python
import rospy
from ardrone.msg import Markers
from ardrone.msg import Marker


def callback(data):
    rospy.loginfo(str(len(data.markers)) + " markers...")

    for marker in data.markers:
        print (str(marker.x) + ", " + str(marker.y))

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("markers", Markers, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()