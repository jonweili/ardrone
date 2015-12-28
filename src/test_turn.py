#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('example_node', anonymous=True)

    # publish commands (send to quadrotor)
    pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    pub_land = rospy.Publisher('/ardrone/land', Empty)
    pub_reset = rospy.Publisher('/ardrone/reset', Empty)
    pub_twist = rospy.Publisher('/cmd_vel', Twist)



    print("ready!")
    rospy.sleep(1.0)

    print("takeoff..")
    pub_takeoff.publish(Empty())

    print ("turn z positive")
    t = Twist()
    t.angular.z = 0.3
    pub_twist.publish(t)

    rospy.sleep(2.0)

    print("land..")
    pub_land.publish(Empty())

    print("done!")
