#!/usr/bin/env python

import rospy
import rospkg
import os
import random
import cv2
import argparse
import sys
import time

from geometry_msgs.msg import Point
from lab5_spawn_block import *
from lab5_blob_search import *
from lab5_header import *
from lab5_func import *
from lab5_coordinate_converter import *

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 130*PI/180.0]

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

num_configs = 5

# Place holder for block world positions
green_world_position = None
yellow_world_position = None

# Final goal positions
# PLEASE USE THE FOLLOWING GOAL
green_world_goal = Point()
green_world_goal.x = 0.3
green_world_goal.y = 0.05
green_world_goal.z = 0.015

yellow_world_goal = Point()
yellow_world_goal.x = 0.3
yellow_world_goal.y = 0.25
yellow_world_goal.z = 0.015

############## Your Code Start Here ##############

"""
TODO: define ROS topic callback funtions for getting the position of blocks
Whenever block_[color]/world publishes info these callback functions are called.
"""
#  image (camera) positionpublisher
# world position subscriber

############### Your Code End Here ###############


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################

############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates (Point)
    target_xw_yw_zw: where to place the block in global coordinates (Point)
    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block
    """

    # global variable1
    # global variable2

    error = 0

    thetas_calc = []
    middle = []
    middle2 = []
    destin = []

    middle = lab_invk(.27, .1, .20, 0)
    move_arm(pub_cmd, loop_rate, middle, vel, accel) # middle position

    thetas_calc = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], 0)
    # print(thetas_calc)
    move_arm(pub_cmd, loop_rate, thetas_calc, vel, accel) # move arm to block
    # print("after move arm")
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1.0)
    if (not digital_in_0):
        error = 1
        return error

    middle2 = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], .25, 0)
    move_arm(pub_cmd, loop_rate, middle2, vel, accel) # middle position

    destin = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], 0)
    move_arm(pub_cmd, loop_rate, destin, vel, accel) # move block to dest 
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(1.0)
    if (digital_in_0):
        error = 0
        return error

    move_arm(pub_cmd, loop_rate, middle, vel, accel) # back to middle 

    return error

############### Your Code End Here ###############

if __name__ == '__main__':

    # Parser
    parser = argparse.ArgumentParser(description='Please specify if a block is taken away or not')
    parser.add_argument('--missing', type=str, default='False')
    parser.add_argument('--image', type=int, default=0)
    args = parser.parse_args()

    # Check parser
    if args.missing.lower() == 'true':
        missing_block = True
    elif args.missing.lower() == 'false':
        missing_block = False
    else:
        print("Invalid argument for missing block, enter True of False")
        sys.exit()

    if args.image < 0 or args.image > 4:
        print("Invalid argument for image.  Enter a number from 0 to 4")
        sys.exit()
    else:
        config_idx = args.image

    # Initialize ROS node
    rospy.init_node('lab5_exec', anonymous=True)

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############

    # Define image position publisher
    # rospy.Publisher(?)

    # Define world position subscriber
    # rospy.Subscriber(?)

    ############## Your Code End Here ###############

    # Initialize ROS pack
    rospack = rospkg.RosPack()

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)

    # Spawn block
    spawn_block(config_idx, missing_block)

    # Get path to image
    lab5_path = rospack.get_path('lab5pkg_py')
    image_path = os.path.join(lab5_path, 'scripts', 'configs', str(config_idx) + '.jpg')
    # Read image
    raw_image = cv2.imread(image_path)

    # Flip the image (Legacy webcam config)
    cv_image = cv2.flip(raw_image, -1)

    # Call blob search (row, col)
    green_image_rc = blob_search(cv_image.copy(), 'green')
    yellow_image_rc = blob_search(cv_image.copy(), 'yellow')

    ############## Your Code Start Here ##############
    # Hint: Remember to delay for your subscriber to update
    # Otherwise you might get None or old images (when running lab 5 multiple times)
    # a short rospy.sleep(time in seconds) after you publish should be sufficient

    # lab 2 move_block()
    start_yellow_xyz = [yellow_image_rc[0], yellow_image_rc[1], yellow_world_goal.z]
    print(yellow_image_rc[0], yellow_image_rc[1])
    #start_green_xyz = [green_x, green_y, green_world_goal.z]

    rospy.sleep(3)

    end_yellow_xyz = [yellow_world_goal.x, yellow_world_goal.y, yellow_world_goal.z]
    #end_green_xyz = [green_world_goal.x, green_world_goal.y, green_world_goal.z]

    gripper_error = 0    

    gripper_error = move_block(pub_command, loop_rate, start_yellow_xyz, end_yellow_xyz, vel, accel)
    if gripper_error:
        print("~~~~~~~NO BLOCK FOUND~~~~~~~")

    # gripper_error = 0
    # gripper_error = move_block(pub_command, loop_rate, start_green_xyz, end_green_xyz, vel, accel)
    # if gripper_error:
    #     print("~~~~~~~NO BLOCK FOUND~~~~~~~")
    
    # gripper_error = 0
    # gripper_error = move_block(pub_command, loop_rate, startg1_xyz, endg1_xyz, vel, accel)
    # if gripper_error:
    #     print("~~~~~~~NO BLOCK FOUND~~~~~~~")

    # gripper_error = 0
    # gripper_error = move_block(pub_command, loop_rate, startg2_xyz, endg2_xyz, vel, accel)   
    # if gripper_error:
    #     print("~~~~~~~NO BLOCK FOUND~~~~~~~")    

    ############## Your Code End Here ###############

    # Move arm to away position
    move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.sleep(1)