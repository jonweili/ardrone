#!/usr/bin/env python
"""
    Publish drone state, using navdata from ardrone, markerdata from a marker publisher and
    the Extended Kalmann Filter.

    I created this to try out the principles from the course: TUMx: AUTONAVx Autonomous Navigation for Flying Robots on EDx

"""

# Python
import time
import numpy as np
import math
import csv

# ROS
import rospy
from ardrone.msg import DroneState
from ardrone.msg import Markers
from ardrone_autonomy.msg import Navdata


class Pose2D:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation

    def inv(self):
        '''
        inversion of this Pose2D object

        :return - inverse of self
        '''
        inv_rotation = self.rotation.transpose()
        inv_translation = -np.dot(inv_rotation, self.translation)

        return Pose2D(inv_rotation, inv_translation)

    def yaw(self):
        from math import atan2
        return atan2(self.rotation[1,0], self.rotation[0,0])

    def __mul__(self, other):
        '''
        multiplication of two Pose2D objects, e.g.:
            a = Pose2D(...) # = self
            b = Pose2D(...) # = other
            c = a * b       # = return value

        :param other - Pose2D right hand side
        :return - product of self and other
        '''
        return Pose2D(np.dot(self.rotation, other.rotation), np.dot(self.rotation, other.translation) + self.translation)



class DroneStateEKF:

    def __init__(self):

        # load marker positions from file
        # TODO: make configurable
        filename = "/home/ros/catkin_ws/src/ardrone/src/christmas_markers.csv"
        self.markers = []
        with open(filename, 'rb') as csvfile:
            marker_reader = csv.reader(csvfile, delimiter=',', quotechar='|')

            for row in marker_reader:
                marker = []
                for col in row:
                    marker.append(float(col))
                self.markers.append(marker)

        self.pub_state = rospy.Publisher('dronestate', DroneState)

        # TODO: tweak values for AR Drone
        # process noise
        pos_noise_std = 0.5
        yaw_noise_std = 0.5
        self.Q = np.array([
            [pos_noise_std*pos_noise_std,0,0],
            [0,pos_noise_std*pos_noise_std,0],
            [0,0,yaw_noise_std*yaw_noise_std]
        ])

        # measurement noise
        z_pos_noise_std = 0.01
        z_yaw_noise_std = 0.03
        self.R = np.array([
            [z_pos_noise_std*z_pos_noise_std,0,0],
            [0,z_pos_noise_std*z_pos_noise_std,0],
            [0,0,z_yaw_noise_std*z_yaw_noise_std]
        ])

        # state vector [x; y; yaw] in world coordinates
        self.x = np.zeros((3,1))

        # 3x3 state covariance matrix
        self.sigma = 0.01 * np.identity(3)

        self.z = 1.
        self.last_rotZ = 0

    def start(self):

        self.last_state_update = time.time()

        self.listener()

        rospy.spin()

    def get_markers(self):
        """ world coordinates for all known markers """

        # TODO: load markers from file

        return self.markers

    def get_marker(self, marker_position_relative):
        """ returns world position for marker closest to estimated position """

        #print "find"
        #print marker_position_relative

        # rotatie matrix (in 2D)
        #print self.x
        R = self.rotation(self.x.item(2))

        # rotatiepunt = quad (global)
        A = np.array([[self.x.item(0)], [self.x.item(1)]])

        #print A

        B_local = np.array([[marker_position_relative[0]], [marker_position_relative[1]]])

        #print B_local

        B_global = R * B_local + A

        #print B_global
        theta_estimated = 0 # dummy, gebtuik ik nog niet

        estimated_marker_position = (B_global.item(0), B_global.item(1,1), theta_estimated)

        closestMarker = None
        smallestError = float("inf")

        #print "est: "
        #print estimated_marker_position

        for marker in self.get_markers():
            error = self.calculate_distance(marker, estimated_marker_position)
            #print error
            if error < smallestError:
                smallestError = error
                closestMarker = marker

        assert closestMarker != None, "Cannot find closest marker"

        return np.array([[closestMarker[0]], [closestMarker[1]]]), closestMarker[2]

    def calculate_distance(self, p1, p2):

        dx = (p1[0] - p2[0])
        dy = (p1[1] - p2[1])


        return math.sqrt(dx*dx + dy*dy)

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

    def normalizeYaw(self, y):
        '''
        normalizes the given angle to the interval [-pi, +pi]
        '''
        while(y > math.pi):
            y -= 2 * math.pi
        while(y < -math.pi):
            y += 2 * math.pi
        return y


    def predictState(self, dt, x, u_linear_velocity, u_yaw_velocity):
        '''
        predicts the next state using the current state and
        the control inputs local linear velocity and yaw velocity
        '''
        x_p = np.zeros((3, 1))
        x_p[0:2] = x[0:2] + dt * np.dot(self.rotation(x[2]), u_linear_velocity)
        x_p[2]   = x[2]   + dt * u_yaw_velocity
        x_p[2]   = self.normalizeYaw(x_p[2])

        return x_p

    def calculatePredictStateJacobian(self, dt, x, u_linear_velocity, u_yaw_velocity):
        '''
        calculates the 3x3 Jacobian matrix for the predictState(...) function
        '''
        s_yaw = math.sin(x[2])
        c_yaw = math.cos(x[2])

        dRotation_dYaw = np.array([
            [-s_yaw, -c_yaw],
            [ c_yaw, -s_yaw]
        ])
        F = np.identity(3)
        #F[0:2, 2] = dt * np.dot(dRotation_dYaw, u_linear_velocity)

        # Misschien iets veranderd in numpy, kolom wordt vanzelf een vector
        # Nu 'handmatig' in de matrix plaatsen

        #print F
        temp = dt * np.dot(dRotation_dYaw, u_linear_velocity)
        F[0, 2] = temp[0]
        F[1, 2] = temp[1]

        return F

    def predictCovariance(self, sigma, F, Q):
        '''
        predicts the next state covariance given the current covariance,
        the Jacobian of the predictState(...) function F and the process noise Q
        '''
        return np.dot(F, np.dot(sigma, F.T)) + Q

    def state_callback(self, t, dt, linear_velocity, yaw_velocity):
        '''
        called when a new odometry measurement arrives approx. 200Hz

        :param t - simulation time
        :param dt - time difference this last invocation
        :param linear_velocity - x and y velocity in local quadrotor coordinate frame (independet of roll and pitch)
        :param yaw_velocity - velocity around quadrotor z axis (independet of roll and pitch)

        :return tuple containing linear x and y velocity control commands in local quadrotor coordinate frame (independet of roll and pitch), and yaw velocity
        '''

        # update state

        self.x = self.predictState(dt, self.x, linear_velocity, yaw_velocity)

        F = self.calculatePredictStateJacobian(dt, self.x, linear_velocity, yaw_velocity)
        self.sigma = self.predictCovariance(self.sigma, F, self.Q);


    def callback_navdata(self, data):
        self.z = data.altd / 1000.

        current_time = time.time()
        dt = current_time - self.last_state_update
        self.last_state_update = current_time

        linear_velocity = np.array([
            [data.vx / 1000.],
            [data.vy / 1000.]
        ])

        #print linear_velocity

        yaw_velocity = ((data.rotZ - self.last_rotZ) / 360. * 2. * math.pi) / dt

        self.last_rotZ = data.rotZ

        self.state_callback(data.tm / 1000000., dt, linear_velocity, yaw_velocity)

        self.publish()



    def predictMeasurement(self, x, marker_position_world, marker_yaw_world):
        '''
        predicts a marker measurement given the current state and the marker position and orientation in world coordinates
        '''
        z_predicted = Pose2D(self.rotation(x[2]), x[0:2]).inv() * Pose2D(self.rotation(marker_yaw_world), marker_position_world);

        return np.array([[z_predicted.translation.item(0), z_predicted.translation.item(1), z_predicted.yaw()]]).T

    def calculatePredictMeasurementJacobian(self, x, marker_position_world, marker_yaw_world):
        '''
        calculates the 3x3 Jacobian matrix of the predictMeasurement(...) function using the current state and
        the marker position and orientation in world coordinates

        :param x - current state 3x1 vector
        :param marker_position_world - x and y position of the marker in world coordinates 2x1 vector
        :param marker_yaw_world - orientation of the marker in world coordinates
        :return - 3x3 Jacobian matrix of the predictMeasurement(...) function
        '''

        # TODO: implement computation of H

        xt = x[0]
        yt = x[1]
        sinphi = math.sin(x[2])
        cosphi = math.cos(x[2])

        xm = marker_position_world[0]
        ym = marker_position_world[1]

        H = np.array([
            [-cosphi, -sinphi, -sinphi*(xm-xt) + cosphi*(ym-yt)],
            [ sinphi, -cosphi, -cosphi*(xm-xt) - sinphi*(ym-yt) ],
            [ 0, 0, -1]
        ])

        return H

    def calculateKalmanGain(self, sigma_p, H, R):
        '''
        calculates the Kalman gain
        '''
        return np.dot(np.dot(sigma_p, H.T), np.linalg.inv(np.dot(H, np.dot(sigma_p, H.T)) + R))

    def correctState(self, K, x_predicted, z, z_predicted):
        '''
        corrects the current state prediction using Kalman gain, the measurement and the predicted measurement

        :param K - Kalman gain
        :param x_predicted - predicted state 3x1 vector
        :param z - measurement 3x1 vector
        :param z_predicted - predicted measurement 3x1 vector
        :return corrected state as 3x1 vector
        '''

        # TODO: implement correction of predicted state x_predicted

        # np.dot(H, x_predicted) => predicted measurement
        corrected_x = x_predicted + np.dot(K, (z - z_predicted))

        return corrected_x

    def correctCovariance(self, sigma_p, K, H):
        '''
        corrects the sate covariance matrix using Kalman gain and the Jacobian matrix of the predictMeasurement(...) function
        '''
        return np.dot(np.identity(3) - np.dot(K, H), sigma_p)

    def measurement_callback(self, marker_position_world, marker_yaw_world, marker_position_relative, marker_yaw_relative):
        '''
        called when a new marker measurement arrives max 30Hz, marker measurements are only available if the quadrotor is
        sufficiently close to a marker

        :param marker_position_world - x and y position of the marker in world coordinates 2x1 vector
        :param marker_yaw_world - orientation of the marker in world coordinates
        :param marker_position_relative - x and y position of the marker relative to the quadrotor 2x1 vector
        :param marker_yaw_relative - orientation of the marker relative to the quadrotor
        '''

        z = np.array([[marker_position_relative.item(0), marker_position_relative.item(1), marker_yaw_relative]]).T
        z_predicted = self.predictMeasurement(self.x, marker_position_world, marker_yaw_world)

        #print z
        #print z_predicted

        H = self.calculatePredictMeasurementJacobian(self.x, marker_position_world, marker_yaw_world)
        #print H
        K = self.calculateKalmanGain(self.sigma, H, self.R)
        #print K

        self.x = self.correctState(K, self.x, z, z_predicted)
        #print self.x
        self.sigma = self.correctCovariance(self.sigma, K, H)
        #print self.sigma


    def callback_markers(self, data):
        #print "#"
        for marker in data.markers:


            m_world = self.get_marker([marker.x, marker.y, marker.theta])
            self.measurement_callback(m_world[0], m_world[1], np.array([[marker.x],[marker.y]]), marker.theta)


        # publish alleen op navdata (is de snelste van de twee, en is er altijd)
        #self.publish()


    def listener(self):

        rospy.Subscriber("/ardrone/navdata", Navdata, self.callback_navdata)
        rospy.Subscriber("markers", Markers, self.callback_markers)

        rospy.spin() # keep running

    def publish(self):
        d = DroneState()
        d.x = self.x[0]
        d.y = self.x[1]
        d.theta = self.x[2]
        d.z = self.z

        rospy.loginfo("drone at: " + str(d.x) + ", " + str(d.y) + ", " + str(d.z)+ " rad: " + str(d.theta))


        self.pub_state.publish(d)


if __name__ == '__main__':


    rospy.init_node('dronestate_gazebo_simulator', anonymous=True)

    rospy.loginfo("Starting EKF dronestate based on navdata and markers")

    ekf = DroneStateEKF()
    ekf.start()

