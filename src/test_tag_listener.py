#!/usr/bin/env python
import rospy
from ardrone_autonomy.msg import Navdata


def callback(data):
    for i in range(data.tags_count):
        print "x: " + str(data.tags_xc[i]) + " y: " + str(data.tags_yc[i])

def listener():

    rospy.init_node('listener', anonymous=True)


    rospy.Subscriber('/ardrone/navdata', Navdata, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()