#!/usr/bin/env python

import sys
import copy
import time
import rospy

import cv2
import os

import numpy as np
from final_header import *
from final_func import *


################ Pre-defined parameters and functions below (can change if needed) ################

# 20Hz
SPIN_RATE = 20  

# UR3 home location
home = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]  

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)  

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False


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

            rospy.loginfo("Goal is reached!")
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
def move_arm(pub_cmd, loop_rate, dest, vel, accel, move_type):
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
    driver_msg.move_type = move_type  # Move type (MoveJ or MoveL)
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

################ Pre-defined parameters and functions above (can change if needed) ################

##========= TODO: Helper Functions =========##

def find_keypoints(image):
    """Gets keypoints from the given image

    Parameters
    ----------
    image : np.ndarray
        The given image (before or after preprocessing)

    Returns
    -------
    keypoints
        a list of keypoints detected in image coordinates
    """

    thresh_min = 60
    thresh_max = 100


    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
    #2. Gaussian blur the image
    blur_img = cv2.GaussianBlur(gray_img, (3,3), 0)
        
    #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
    Gx = cv2.Sobel(blur_img, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=3)
    Gy = cv2.Sobel(blur_img, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=3)
        
    Gx = cv2.convertScaleAbs(Gx)
    Gy = cv2.convertScaleAbs(Gy)

    # m = cv2.magnitude(Gx,Gy)

    # m = cv2.normalize(m,None,0.,255.,cv2.NORM_MINMAX,cv2.CV_8U)
        
    #4. Use cv2.addWeighted() to combine the results
    sobel_img = cv2.addWeighted(Gx, 0.5, Gy, 0.5, 0)
        
    #5. Convert each pixel to unint8, then apply threshold to get binary image
    binary_output = cv2.threshold(sobel_img, thresh_min, thresh_max, cv2.THRESH_BINARY)[1]

    cv2.imshow("edge", binary_output)
    cv2.waitKey(0)

    contours, hierarchy = cv2.findContours(binary_output,  
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    print("Number of Contours found = " + str(len(contours)))

    print(contours)

    pt_array = []
    pts = []
    for contour in contours:
        for index,pt in enumerate(contour):
            if(index%10 == 0):
                pts.append(pt[0])
            # pts.append(pt[0])
        pt_array.append(np.array(pts))
        pts = []
    reduced_contours = pt_array
    # print(contours)
    # print(len(contours[0]))
    print(len(reduced_contours))
    print(reduced_contours)


    img = cv2.cvtColor(binary_output,cv2.COLOR_GRAY2BGR)
    cv2.drawContours(img, contours, -1, (0, 255, 0), 3) 
  
    cv2.imshow('Contours', img) 
    cv2.waitKey(0) 

    img = cv2.cvtColor(binary_output,cv2.COLOR_GRAY2BGR)
    for reduced_contour in reduced_contours:
        for i in range(len(reduced_contour)):
            if(i+1 != len(reduced_contour)):
                cv2.line(img, reduced_contour[i],reduced_contour[i+1], (0, 255, 0), 1) 

    cv2.imshow('reduced Contours', img) 
    cv2.waitKey(0) 

    cv2.destroyAllWindows() 


    keypoints = []
    return keypoints

def IMG2W(row, col, image):
    """Transform image coordinates to world coordinates

    Parameters
    ----------
    row : int
        Pixel row position
    col : int
        Pixel column position
    image : np.ndarray
        The given image (before or after preprocessing)

    Returns
    -------
    x : float
        x position in the world frame
    y : float
        y position in the world frame
    """
    x, y = 0.0, 0.0
    return x, y

def draw_image(world_keypoints):
    """Draw the image based on detecte keypoints in world coordinates

    Parameters
    ----------
    world_keypoints:
        a list of keypoints detected in world coordinates
    """
    pass


"""
Program run from here
"""
def main():
    global home
    # global variable1
    # global variable2

    # Initialize ROS node
    rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    # Velocity and acceleration of the UR3 arm
    vel = 4.0
    accel = 4.0
    # move_arm(pub_command, loop_rate, home, vel, accel, 'J')  # Move to the home position

    ##========= TODO: Read and draw a given image =========##
    cwd = os.getcwd()
    print(cwd)
    input_img = cv2.imread(cwd+"/images/turkey.png")
    # cv2.imshow("original", input_img)
    # cv2.waitKey(0)

    keypoints = find_keypoints(image=input_img)
     


    # move_arm(pub_command, loop_rate, home, vel, accel, 'J')  # Return to the home position
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
