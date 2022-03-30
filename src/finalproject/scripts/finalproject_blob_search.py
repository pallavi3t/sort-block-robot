#!/usr/bin/env python

import cv2
import numpy as np
from geometry_msgs.msg import Point
import math

# Params for camera calibration
theta = -0.09
beta = 730.0
tx=-0.297435628054
ty=-0.0763807206584

# Params for camera
w = 640
h = 480

# Params for block
block_height = 0.015

# Function that converts image coord to world coord
def IMG2W(r,c):
    global theta
    global beta
    global tx
    global ty

    ################################ Your Code Start Here ################################
    # Given theta, beta, tx, ty, calculate the world coordinate of r,c namely xw, yw

    xcR = (r - (h/2)) / beta
    ycR = (c - (w/2)) / beta

    camera_points = np.array([[xcR],[ycR], [1]])
    t = np.array([[tx],[ty], [1]]) # find transformation matrix of x and y

    R = np.array([[math.cos(theta),-1*math.sin(theta),0],[math.sin(theta),math.cos(theta),0],[0,0,1]])
    rot_cam = np.linalg.inv(R)

    world_points = np.matmul(rot_cam, (camera_points - t))

    xw = world_points[0][0]
    yw = world_points[1][0]

    # check function with nikhils input output values

    ################################# Your Code End Here #################################
    return xw, yw


def blob_search(image_raw, color):

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    
    # Edit Below
    ##################################
    # Replace these with your values for each color

    lower = (5, 150, 100)
    upper = (15, 255, 255)

    if (color == "yellow"):
        lower = (25, 100, 125)
        upper = (30, 255, 255)
    elif (color == "green"):
        lower = (40, 100, 100)
        upper = (70, 255, 200) 
    
    # Edit Above
    ##################################

    mask_image =  cv2.inRange(hsv_image, lower, upper)
    #cv2.imshow("mask", mask_image)
    #cv2.waitKey(0)    

    # Edit Below
    ##################################
    # Setup SimpleBlobDetector parameters by editing the following code:
    params = cv2.SimpleBlobDetector_Params()

    # Filter by Color
    params.filterByColor = True
    params.blobColor = 255

    # Filter by Area.
    params.filterByArea = False

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.3
    params.maxCircularity = 0.5

    # Filter by Inertia
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # Edit Above
    ##################################

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect keypoints
    keypoints = detector.detect(mask_image)
    i = len(keypoints)
    if i == 0:
        print("No blobs found... ")
        r = None
        c = None
    elif i == 1:
        print("One blob found... Yay!")
        keypoint = keypoints[0]
        c = keypoint.pt[0]
        r = keypoint.pt[1]
    else:
        print("{} blobs found, only passing the first...".format(i) )
        keypoint = keypoints[0]

        # Get x and y
        c = keypoint.pt[0]
        r = keypoint.pt[1]

    im_with_keypoints = image_raw

    if len(keypoints) == 0:
        im_with_keypoints = image_raw
    else:
        # Feel free to use these as the color that you draw your keypoints and circle
        if color == 'yellow':
            draw_color = (255, 0, 0)
        else:
            draw_color = (255, 0, 255)
    
    # Edit Below
    ##################################
    # Edit below to mark keypoints and draw a circle around the block.
        # Draw a circle around the detected block
        im_with_keypoints = cv2.circle(image_raw, (int(c), int(r)), radius=20, color=(255, 0, 0), thickness=2)

        # Draw the keypoints on the detected block
        im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, keypoints, mask_image, color=(255,0,0))
    
    # Edit Above
    ##################################
    
    # Show masked image
    im_mask = cv2.cvtColor(mask_image, cv2.COLOR_GRAY2BGR)
    cv2.namedWindow("Masked Image")
    cv2.imshow("Masked Image", im_mask)

    # Note to students for pressing enter to continue
    im_with_keypoints = cv2.putText(im_with_keypoints, 'Press Enter to Continue', (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    cv2.namedWindow("Press Enter to Continue")
    cv2.imshow("Press Enter to Continue", im_with_keypoints)

    while True:
        key = cv2.waitKey(0)
        if key == 13:
            cv2.destroyAllWindows()
            break

    if (r == None or c == None):
        return 
    else: 
        return IMG2W(r,c)