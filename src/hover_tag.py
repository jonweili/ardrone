#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty

from geometry_msgs.msg import Pose, Twist
from ardrone.msg import Markers
from ardrone.msg import Marker

last_error_x = 0
last_error_y = 0
last_error_z = 0


def fly_to_marker(marker_data):
    global last_error_x
    global last_error_y

    if len(marker_data.markers) > 0:

        target = marker_data.markers[0]

        error_x = target.x
        error_y = target.y

        delta_error_x = error_x - last_error_x
        delta_error_y = error_y - last_error_y


        p = 0.4
        d = 0.2

        move_x = max(-1, min(p * error_x + d * delta_error_x, 1));
        move_y = max(-1, min(p * error_y + d * delta_error_y, 1));

        print str(move_x) + ", " + str(move_y)

        t = Twist()
        t.linear.x = move_x
        t.linear.y = move_y

        pub_velocity.publish(t)

        last_error_x = error_x
        last_error_y = error_y



if __name__ == '__main__':
    rospy.init_node("example_node", anonymous=True)

    # publish commands (send to quadrotor)
    pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    pub_land = rospy.Publisher('/ardrone/land', Empty)
    pub_reset = rospy.Publisher('/ardrone/reset', Empty)
    pub_velocity = rospy.Publisher('/cmd_vel', Twist)




    pub_reset.publish(Empty())
    print("ready!")
    rospy.sleep(1.0)

    print("takeoff..")
    pub_takeoff.publish(Empty())

    rospy.sleep(5.0)

    rospy.Subscriber('markers', Markers, fly_to_marker)

    rospy.sleep(10.0)

    print("land..")
    pub_land.publish(Empty())

    print("done!")
