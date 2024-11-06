#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.0256 # rad
beta = 75/0.1 # pixels/Meter
tx = 0.0
ty = 0.0

# Function that converts image coord to world coord
def IMG2W(col, row):

    global theta,beta

    base_y = 262
    base_x = 14
    
    img_x = row
    img_y = col

    Tcw = np.array([
        [np.cos(-theta), -np.sin(-theta), base_x],
        [np.sin(-theta), np.cos(theta), base_y],
        [0,0,1]
    ])

    Twc = np.linalg.inv(Tcw)
    img_c = np.array([[img_x, img_y, 1]]).T
    img_w = Twc@img_c/beta
    return([img_w[0],img_w[1]])

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True

    params.minArea = 200
    params.maxArea = 1000


    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False


    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    b_lower = (110,50,50)     # blue lower
    b_upper = (130,255,255)   # blue upper

    g_lower = (80/2.0, 100, 100)
    g_upper = (160/2.0, 255, 255)

    p_lower = (0, 100, 100)
    p_upper = (5, 255, 255)


    o_lower = (5, 100, 100)
    o_upper = (20, 255, 255)


    if(color=="Green"):
        lower = g_lower
        upper = g_upper
    elif(color == "Pink"):
        lower = p_lower
        upper = p_upper
    else:
        lower = b_lower
        upper = b_upper

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block

    # out_img = cv2.UMat()
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, None, color=(255, 255, 255))

    # cv2.imshow("window",im_with_keypoints)
    # cv2.waitKey(3)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))
            # print(i, blob_image_center[i][0], blob_image_center[i][1])
            # print()
            # xy = IMG2W(blob_image_center[i][0], blob_image_center[i][1])
            # print(i,  xy[0], xy[1])
            # print()
    
            # print(blob_image_center[i][0])

    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
