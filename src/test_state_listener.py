#!/usr/bin/env python
import rospy
from ardrone.msg import DroneState


def callback(data):
    rospy.loginfo(" state:")

    print (str(data.x) + ", " + str(data.y) + ", " + str(data.z) + " rad: " + str(data.theta))

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("dronestate", DroneState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()