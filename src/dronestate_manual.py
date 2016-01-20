#!/usr/bin/env python
"""
    Publish drone state, dummy: gets data from user

"""

# Python
import time
import numpy as np
import math
import pygame
from pygame.locals import *
import sys


# ROS
import rospy
from ardrone.msg import DroneState
from ardrone_autonomy.msg import Navdata

# Config
SCREEN_WIDTH = 600
SCREEN_HEIGHT = 600

MAX_X = 2.
MAX_Y = 2.
MIN_X = -2.
MIN_Y = -2.

MANUAL_HEIGHT = False # if set to False height comes from Navdata

quad_x = 0.
quad_y = 0.
quad_z = 0.
quad_theta = 0.

def rot_center(image, angle):
    """rotate an image while keeping its center and size"""
    orig_rect = image.get_rect()
    rot_image = pygame.transform.rotate(image, angle)
    rot_rect = orig_rect.copy()
    rot_rect.center = rot_image.get_rect().center
    rot_image = rot_image.subsurface(rot_rect).copy()
    return rot_image

def callback_navdata(data):

    global quad_z

    quad_z = data.altd / 1000.



def pub_state():
    global quad_x, quad_y, quad_z, quad_theta

    droneState = DroneState()

    droneState.x = quad_x
    droneState.y = quad_y
    droneState.z = quad_z
    droneState.theta = quad_theta

    # publish drone state

    rospy.loginfo("drone at: " + str(droneState.x) + ", " + str(droneState.y) + ", " + str(droneState.z)+ " rad: " + str(droneState.theta))


    pub_markers.publish(droneState)


def talker():
    global quad_x, quad_y, quad_z, quad_theta

    # start pygame interface
    rate = rospy.Rate(10) # 10hz

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    clock = pygame.time.Clock()

    background_color = (255, 255, 255)

    # TODO: check eenheden!!!

    quad_x = 0.
    quad_y = 0.
    quad_z = 1.0
    quad_theta = 0.

    # Fill background
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((250, 250, 250))

    font = pygame.font.Font(None, 36)

    image = pygame.image.load("/home/ros/catkin_ws/src/ardrone/src/arrow-right_blue.png") #.convert_alpha()

    [x, y] = (SCREEN_WIDTH / 2 - 30, SCREEN_HEIGHT / 2 - 25)

    rot_image = pygame.transform.rotate(image, float(0))

    update = True

    while not rospy.is_shutdown():

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

            if event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                x = pos[0] - 30
                y = pos[1] - 25

                quad_x = (MIN_X + pos[0] / float(SCREEN_WIDTH) * float(MAX_X - MIN_X))
                quad_y = (MIN_Y + pos[1] / float(SCREEN_HEIGHT) * float(MAX_Y - MIN_Y)) * -1 # y reversed
                update = True


        keys = pygame.key.get_pressed()

        if (keys[K_LEFT]):
            #print "left"
            quad_theta = quad_theta - 0.01
            rot_image = rot_center(image, float(quad_theta * -360.))
            update = True
        if (keys[K_RIGHT]):
            quad_theta = quad_theta + 0.01
            rot_image = rot_center(image, float(quad_theta * -360.))
            update = True

        if MANUAL_HEIGHT:
            if (keys[K_DOWN]):
                quad_z = quad_z - 0.01
                update = True
            if (keys[K_UP]):
                quad_z = quad_z + 0.01
                update = True

        screen.fill(background_color)

        # Display some text

        text4 = font.render("theta: " + str(quad_theta * 360.), 1, (10, 10, 10))
        text1 = font.render("x: " + str(quad_x), 1, (10, 10, 10))
        text2 = font.render("y: " + str(quad_y), 1, (10, 10, 10))
        text3 = font.render("z: " + str(quad_z), 1, (10, 10, 10))

        background.fill((250, 250, 250))
        background.blit(text1, (10,10))
        background.blit(text2, (10,30))
        background.blit(text3, (10,50))
        background.blit(text4, (10,70))

        pygame.draw.line(background, (0, 0, 0), (SCREEN_WIDTH / 2, 0), (SCREEN_WIDTH / 2,SCREEN_HEIGHT), 5)
        pygame.draw.line(background, (0, 0, 0), (0,SCREEN_HEIGHT / 2), (SCREEN_WIDTH,SCREEN_HEIGHT / 2), 5)

        background.blit(rot_image, (x, y))



        # Blit everything to the screen
        screen.blit(background, (0, 0))

        clock.tick(100)

        pygame.display.flip()

        #if update:
        pub_state()

        update = False

        rate.sleep()

    sys.exit()


if __name__ == '__main__':

    rospy.Subscriber('/ardrone/navdata', Navdata, callback_navdata)

    pub_markers = rospy.Publisher('dronestate', DroneState)

    rospy.init_node('dronestate_manual', anonymous=True)

    rospy.loginfo("Starting simulated dronestate (Manual)")

    talker()
