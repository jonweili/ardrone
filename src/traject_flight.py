#!/usr/bin/env python
"""
    Fly trajectory with AR Drone

    PID control, using targets from file and published dronestate

"""

# Python
import time
import numpy as np
import math
import csv

# ROS
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Vector3
from ardrone.msg import DroneState


class Trajectory:
    def __init__(self):


        # load trajectory from file
        # TODO: make configurable (sys.argv contains list of arguments)
        filename = "/home/ros/catkin_ws/src/ardrone/src/hover_traject.csv"
        self.targets = []
        with open(filename, 'rb') as csvfile:
            trajectory_reader = csv.reader(csvfile, delimiter=',', quotechar='|')

            for row in trajectory_reader:
                target = []
                for col in row:
                    target.append(float(col))
                self.targets.append(target)

        self.last_state_update = time.time()

        self.target_position = [0,0]
        self.current_target = -1

        self.pub_velocity = rospy.Publisher('/cmd_vel', Twist)

        self.in_flight = False

        # PID constants for x, y, z and theta

        self.KP = np.array([
            0.1,
            0.1,
            0.2,
            0.0
        ])

        self.KI = np.array([
            0.000,
            0.000,
            0.000,
            0.000
        ])

        self.KD = np.array([
            0.000,
            0.000,
            0.0000,
            0.00000
        ])

        # Store running error for I and D

        self.last_error = np.array([
            0.,
            0.,
            0.,
            0.
        ])

        self.total_error = np.array([
            0.,
            0.,
            0.,
            0.
        ])


    def start(self):
        self.last_state_update = time.time()
        self.get_next_target()
        self.x = np.zeros((4,1))

        self.in_flight = True

        self.listener()

        r = rospy.Rate(10) #10hz

        while not rospy.is_shutdown() and self.in_flight:
            r.sleep()


    def get_next_target(self):


        if (self.current_target < (len(self.targets)-1)):
            self.current_target = self.current_target + 1
            self.target_position = self.targets[self.current_target]

            print "new target: " + str(self.target_position[0]) + ", " + str(self.target_position[1])
        else:
            print "no targets left... goal reached"

            # TODO: hover
            #self.in_flight = False
            # Now: cycle
            self.current_target = 0
            self.target_position = self.targets[self.current_target]

    # control functie toevoegen, aanroepen vanaf state callback
    def state_control(self):

        # return control in local coordinate frame

        #position = (np.dot(self.H, self.x))


        error = np.array([
            self.target_position[0] - self.x[0],
            self.target_position[1] - self.x[1]
        ])

        #print self.target_position[0]
        #print error

        error_z = self.target_position[2] - self.x[2]

        error_size = math.sqrt(error.item((0,0))*error.item((0,0)) + error.item((1,0))*error.item((1,0)) + error_z*error_z)


        # TODO: include theta?
        if (error_size < 0.25):
            self.get_next_target()

        u_world = error
        u_robot = np.dot(self.rotation(-1*self.x[3]), u_world)



        # TODO: correct heading error to shortest turn
        error_theta = self.target_position[3] - self.x[3]

        current_error = np.array([
            u_robot.item(0,0),
            u_robot.item(1,0),
            error_z,
            error_theta
        ])

        return current_error

    def rotation(self, yaw):
        '''
        create 2D rotation matrix from given angle
        '''
        s_yaw = math.sin(yaw)
        c_yaw = math.cos(yaw)

        return np.array([
            [c_yaw, -s_yaw],
            [s_yaw,  c_yaw]
        ])

    def callback_dronestate(self, data):

        if not self.in_flight:
            return
        current_time = time.time()

        dt = current_time - self.last_state_update

        #u = self.state_callback(data.tm, dt, linear_velocity, data.rotZ)

        self.x[0] = data.x
        self.x[1] = data.y
        self.x[2] = data.z
        self.x[3] = data.theta


        current_error = self.state_control()

        delta_error = (current_error - self.last_error) / dt
        self.total_error = self.total_error + (current_error * dt)

        u = self.KP * current_error + self.KD * delta_error + self.KI * self.total_error

        #print u

        t = Twist()

        t.linear.x = max(-1, min(u[0], 1));
        t.linear.y = max(-1, min(u[1], 1));
        t.linear.z = max(-1, min(u[2], 1));

        # probeersel, hover als je bijna stil moet staan
        min_speed = 0.00
        if abs(t.linear.x) < min_speed:
            t.linear.x = 0
        if abs(t.linear.y) < min_speed:
            t.linear.y = 0
        if abs(t.linear.z) < min_speed:
            t.linear.z = 0


        # TODO: also correct theta (doesn't work => calculation error or drone to inaccurate?)
        #t.angular.z = u[3]




        self.pub_velocity.publish(t)

        print "nav"
        print u
        print t

        self.last_error = current_error
        self.last_update_navdata = current_time


    def listener(self):

        rospy.Subscriber("dronestate", DroneState, self.callback_dronestate)





if __name__ == '__main__':

    rospy.init_node('tag_simulator', anonymous=True)

    #pub_flattrim = rospy.Service('/ardrone/flattrim',None, None)

    #exit()

    # publish commands (send to quadrotor)
    pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    pub_land = rospy.Publisher('/ardrone/land', Empty)
    pub_reset = rospy.Publisher('/ardrone/reset', Empty)

    print("ready!")
    rospy.sleep(1.0)

    print("takeoff..")
    pub_takeoff.publish(Empty())
    rospy.sleep(4.0)

    trajectory = Trajectory()
    trajectory.start()


    rospy.sleep(2.0)


    print("land..")
    pub_land.publish(Empty())

    print("done!")
