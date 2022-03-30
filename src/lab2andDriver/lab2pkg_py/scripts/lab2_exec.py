#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 1
# Q11 = [120*pi/180.0, -56*pi/180.0, 124*pi/180.0, -158*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q12 = [120*pi/180.0, -64*pi/180.0, 123*pi/180.0, -148*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q13 = [120*pi/180.0, -72*pi/180.0, 120*pi/180.0, -137*pi/180.0, -90*pi/180.0, 0*pi/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here DONE ##############
"""
TODO: Initialize Q matrix
"""

# Hanoi tower initial position
Q00 = [154.46*pi/180.0, -82.42*pi/180.0, 90.79*pi/180.0, -100.62*pi/180.0, -92.06*pi/180.0, 0*pi/180.0]

# Hanoi tower location 1 
Q11 = [155.31*pi/180.0, -54.53*pi/180.0, 126.07*pi/180.0, -165.33*pi/180.0, -86.64*pi/180.0, 80.60*pi/180.0]
Q12 = [155.41*pi/180.0, -64.29*pi/180.0, 121.40*pi/180.0, -146.44*pi/180.0, -91.06*pi/180.0, 72.92*pi/180.0]
Q13 = [149.07*pi/180.0, -66.44*pi/180.0, 114.12*pi/180.0, -140.79*pi/180.0, -90.31*pi/180.0, 4.48*pi/180.0]

# Hanoi tower location 2 
Q21 = [162.49*pi/180.0, -54.03*pi/180.0, 126.61*pi/180.0, -167.08*pi/180.0, -85.89*pi/180.0, 80.61*pi/180.0]
Q22 = [162.03*pi/180.0, -64.24*pi/180.0, 121.38*pi/180.0, -145.79*pi/180.0, -86.84*pi/180.0, 73.48*pi/180.0]
Q23 = [155.83*pi/180.0, -66.05*pi/180.0, 113.55*pi/180.0, -139.26*pi/180.0, -88.49*pi/180.0, 6.50*pi/180.0]

# Hanoi tower location 3
Q31 = [171.15*pi/180.0, -53.22*pi/180.0, 124.13*pi/180.0, -165.76*pi/180.0, -86.79*pi/180.0, 80.64*pi/180.0]
Q32 = [169.60*pi/180.0, -63.08*pi/180.0, 121.27*pi/180.0, -148.21*pi/180.0, -85.94*pi/180.0, 73.48*pi/180.0]
Q33 = [165.82*pi/180.0, -64.14*pi/180.0, 110.38*pi/180.0, -140.14*pi/180.0, -92.08*pi/180.0, 0.91*pi/180.0]

Q = [ [Q00, Q00, Q00], \
      [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############

############## Your Code Start Here DONE ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def gripper_input_callback(msg):

    global current_gripper_input_set

    if (msg.DIGIN == True):
        current_gripper_input_set = True
    else:
        current_gripper_input_set = False

############### Your Code End Here ###############


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

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


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
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here DONE ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".

    error = 0

    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height - 1], 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1.0)
    if (not current_gripper_input_set):
        error = 1
        return error
    move_arm(pub_cmd, loop_rate, Q[0][0], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height - 1], 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(1.0)
    if (current_gripper_input_set):
        error = 0
        return error
    move_arm(pub_cmd, loop_rate, Q[0][0], 4.0, 4.0)

    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here DONE ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripper_input = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_input_callback)


    ############### Your Code End Here ###############


    ############## Your Code Start Here  DONE ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    start_pos = 0
    end_pos = 0
    
    loop_count = 1

    # while(not input_done):
    #     input_string = raw_input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
    #     print("You entered " + input_string + "\n")

    #     if(int(input_string) == 1):
    #         input_done = 1
    #         loop_count = 1
    #     elif (int(input_string) == 2):
    #         input_done = 1
    #         loop_count = 2
    #     elif (int(input_string) == 3):
    #         input_done = 1
    #         loop_count = 3
    #     elif (int(input_string) == 0):
    #         print("Quitting... ")
    #         sys.exit()
    #     else:
    #         print("Please just enter the character 1 2 3 or 0 to quit \n\n")

    while (input_done < 2):
        input_string_start = raw_input("Enter start position of Tower of Hanoi <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string_start + " as starting position \n")

        input_string_end = raw_input("Enter end position of Tower of Hanoi <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string_end + " as ending position \n")

        # setting start position 
        if(int(input_string_start) == 1):
            input_done = 1
            start_pos = 1
        elif (int(input_string_start) == 2):
            input_done = 1
            start_pos = 2
        elif (int(input_string_start) == 3):
            input_done = 1
            start_pos = 3
        elif (int(input_string_start) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

        # setting end position 
        if(int(input_string_end) == 1):
            input_done = 2
            end_pos = 1
        elif (int(input_string_end) == 2):
            input_done = 2
            end_pos = 2
        elif (int(input_string_end) == 3):
            input_done = 2
            end_pos = 3
        elif (int(input_string_end) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

        print(start_pos, end_pos)


    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here DONE ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    # while(loop_count > 0):

    #     move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    #     rospy.loginfo("Sending goal 1 ...")
    #     move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

    #     gripper(pub_command, loop_rate, suction_on)
    #     # Delay to make sure suction cup has grasped the block
    #     time.sleep(1.0)

    #     rospy.loginfo("Sending goal 2 ...")
    #     move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

    #     rospy.loginfo("Sending goal 3 ...")
    #     move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

    #     loop_count = loop_count - 1

    # 

    # Set values 
    intermediate_pos = 0
    gripper_error = 0

    if ((start_pos == 1 and end_pos == 3) or (start_pos == 3 and end_pos == 1)):
        intermediate_pos = 2
    elif ((start_pos == 2 and end_pos == 3) or (start_pos == 3 and end_pos == 2)):
        intermediate_pos = 1
    else:
        intermediate_pos = 3

    # A3 -> C1
    if (not gripper_error):
        gripper_error = move_block(pub_command, loop_rate, start_pos, 3, end_pos, 1)

    # A2 -> B1
    if (not gripper_error):
        gripper_error = move_block(pub_command, loop_rate, start_pos, 2, intermediate_pos, 1)

    # C1 -> B2
    if (not gripper_error):
        gripper_error = move_block(pub_command, loop_rate, end_pos, 1, intermediate_pos, 2)

    # A1 -> C1
    if (not gripper_error):
        gripper_error = move_block(pub_command, loop_rate, start_pos, 1, end_pos, 1)

    # B2 -> A1
    if (not gripper_error):
        gripper_error = move_block(pub_command, loop_rate, intermediate_pos, 2, start_pos, 1)

    # B1 -> C2
    if (not gripper_error):
        gripper_error = move_block(pub_command, loop_rate, intermediate_pos, 1, end_pos, 2)

    # A1 -> C3 
    if (not gripper_error):
        gripper_error = move_block(pub_command, loop_rate, start_pos, 1, end_pos, 3)

    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
