#!/usr/bin/env python3

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
Q01 = [170.91*pi/180.0, -49.57*pi/180.0, 104*pi/180.0, -142.85*pi/180.0, -92.42*pi/180.0, 1.09*pi/180.0]
Q02 = [170.91*pi/180.0, -55.20*pi/180.0, 102*pi/180.0, -135.62*pi/180.0, -92.43*pi/180.0, 1.09*pi/180.0]
Q03 = [170.91*pi/180.0, -60.69*pi/180.0, 100.18*pi/180.0, -127.89*pi/180.0, -92.43*pi/180.0, 1.09*pi/180.0]
# Hanoi tower location 2
Q11 = [180.78*pi/180.0, -47.34*pi/180.0, 99.15*pi/180.0, -139.1*pi/180.0, -92.1*pi/180.0, 11.2*pi/180.0]
Q12 = [180.78*pi/180.0, -52.79*pi/180.0, 97*pi/180.0, -132.49*pi/180.0, -92.11*pi/180.0, 11.2*pi/180.0]
Q13 = [180.78*pi/180.0, -57.57*pi/180.0, 98.85*pi/180.0, -133.3*pi/180.0, -92.11*pi/180.0, 11.18*pi/180.0]
# Hanoi tower location 3
Q21 = [190*pi/180.0, -44.15*pi/180.0, 90.91*pi/180.0, -134.42*pi/180.0, -91.75*pi/180.0, 20.42*pi/180.0]
Q22 = [190*pi/180.0, -49.16*pi/180.0, 89.9*pi/180.0, -128.41*pi/180.0, -91.75*pi/180.0, 20.41*pi/180.0]
Q23 = [190*pi/180.0, -53.87*pi/180.0, 87.55*pi/180.0, -121.16*pi/180.0, -91.75*pi/180.0, 20.41*pi/180.0]
# Hanoi tower hover point
Q_hover0 = [170.4*pi/180.0, -71.*pi/180.0, 89.5*pi/180.0, -107.02*pi/180.0, -92.43*pi/180.0, 0.83*pi/180.0]

Q_hover1 = [179.82*pi/180.0, -68.02*pi/180.0, 85.12*pi/180.0, -105.24*pi/180.0, -92.3*pi/180.0, 10.2*pi/180.0]

Q_hover2 = [189.54*pi/180.0, -62.4*pi/180.0, 76.57*pi/180.0, -101.93*pi/180.0, -91.8*pi/180.0, 20*pi/180.0]


thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q01, Q02, Q03, Q_hover0], \
      [Q11, Q12, Q13, Q_hover1], \
      [Q21, Q22, Q23, Q_hover2]]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def cb_gripper(msg):
    global analog_in_0
    analog_in_0 = msg.AIN0


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


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q
    global suction_on
    global suction_off
    global analog_in_0

    ### Hint: Use the Q array to map out your towers by location and "height".

    error = 0

    error = move_arm(pub_cmd,loop_rate,Q[start_loc][3],4,4)
    error = move_arm(pub_cmd,loop_rate,Q[start_loc][start_height],4,4)
    error = gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1)
    if(analog_in_0 < 1.8):
        error = gripper(pub_cmd, loop_rate, suction_off)
        sys.exit()
    error = move_arm(pub_cmd,loop_rate,Q[start_loc][3],4,4)
    error = move_arm(pub_cmd,loop_rate,Q[end_loc][3],4,4)
    error = move_arm(pub_cmd,loop_rate,Q[end_loc][end_height],4,4)
    error = gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(1)
    error = move_arm(pub_cmd,loop_rate,Q[end_loc][3],4,4)


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

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, cb_gripper)


    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    loop_count = 0
    source = 0
    buffer = 1
    des = 2
    while(not input_done):
        input_string = input("Enter [source, destination] or q to quit> ")
        print("You entered " + input_string + "\n")
        # "[1 3]"
        if (input_string == 'q'):
            print("Quitting... ")
            sys.exit()
        source = int(input_string[1])
        des = int(input_string[3])
        buffer = 3 - des - source

        if(source == buffer or source == des or des == buffer):
            print("wrong input, please enter again!")
            input_done = 0
        elif(source > 2 or buffer > 2 or des > 2):
            print("wrong input, please enter again!")
            input_done = 0
        else:
            input_done = 1
        # elif (int(input_string) == 2):
        #     input_done = 1
        #     loop_count = 2
        # elif (int(input_string) == 3):
        #     input_done = 1
        #     loop_count = 3
        # else:
        #     print("Please just enter the character 1 2 3 or 0 to quit \n\n")





    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    # init position
    move_arm(pub_command, loop_rate, Q_hover1, 4.0, 4.0)

    # 1. S_3 -> D_1
    move_block(pub_command,loop_rate,source,2,des,0)

    move_block(pub_command,loop_rate,source,1,buffer,0)

    move_block(pub_command,loop_rate,des,0,buffer,1)

    move_block(pub_command,loop_rate,source,0,des,0)

    move_block(pub_command,loop_rate,buffer,1,source,0)

    move_block(pub_command,loop_rate,buffer,0,des,1)

    move_block(pub_command,loop_rate,source,0,des,2)


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

    # gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
