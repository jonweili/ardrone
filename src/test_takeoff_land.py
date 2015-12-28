#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Vector3
 
if __name__ == '__main__':
    rospy.init_node('example_node', anonymous=True)
 
    # publish commands (send to quadrotor)
    pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    pub_land = rospy.Publisher('/ardrone/land', Empty)
    pub_reset = rospy.Publisher('/ardrone/reset', Empty)
 
    print("ready!")
    rospy.sleep(1.0)
 
    print("takeoff..")
    pub_takeoff.publish(Empty())

    rospy.sleep(3.0)

    #t = Twist()
    #t.angular.z = 0.1

    #pub_velocity = rospy.Publisher('/cmd_vel', t)


    rospy.sleep(2.0)
 
    print("land..")
    pub_land.publish(Empty())
 
    print("done!")
