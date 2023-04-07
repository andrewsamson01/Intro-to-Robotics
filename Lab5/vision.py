"""
John Ripple and Andrew Samson
MEGN 441 Lab 5
Aruco Detection
"""

import cv2 as cv
import numpy as np
from Brains import transmitToArduino
# import argparse
# import imutils
# import sys

selected_coordinates = (0,0) #globals because we need it
def click_event(event, x, y, flags, params):
    if event == cv.EVENT_LBUTTONDOWN:
        #was clicked
        selected_coordinates = (x, y)

def moveBottoPoint(selectedPlace, botPose):
    #my thoughtis its formatted selectedPlace = (x coordinate, ycoordinate)relative to world frme
    #botPose = (x coordinate, y coordinate, w angle comp to world x axis) relative to world frame
    #get vector of bot loc to selected place
    bot_vector = ((selectedPlace(0) - botPose(0)), (selectedPlace(1) - botPose(1)))
    #determine that angle
    angle = np.arctan(bot_vector(1)/bot_vector(0))
    
    #compare that angle to w of bot

    #turn accordingly so we face point (function)

    #move straight
    
    z = 0

def calculatePWMS():
    
    return 255, 255

def getPoses():
    x,y,z,w = 0,0,0,0
    return x,y,z, w

def main():
    scale = 1
    cap = cv.VideoCapture(0, cv.CAP_DSHOW)  # this is the magic!
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920/scale)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080/scale)
    data = np.load('camera_distort_matrices.npz')

    # Camera parameters
    K = data['mtx']

    # Real world lengths in cm
    MARKER_LENGTH = 15/scale

    # Get the pattern dictionary for 7X7 markers, with ids 0 through 99.
    arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_100)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
            # Detect any aruco markers in the image
        corners, ids, _ = cv.aruco.detectMarkers(image=frame, dictionary=arucoDict)
        if ids is not None:
            cv.aruco.drawDetectedMarkers(image=frame, corners=corners, ids=ids, borderColor=(0, 0, 255))
            # Get the pose of all markers in the image
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners=corners, markerLength=MARKER_LENGTH, cameraMatrix=K, distCoeffs=None)
            # Get the pose of the first detected marker with respect to the camera.
            print(tvecs[0])
            
            for i in range(0, len(rvecs)):
                rvec_m_c = rvecs[i]  # This is a 1x3 rotation vector
                tm_c = tvecs[i]  # This is a 1x3 translation vector
                rotation_m_c = cv.Rodrigues(rvec_m_c[0])  # Gets 3x3 rotation matrix from 1x3 rotation vector
                M_ext = np.block([[rotation_m_c[0], tm_c.T]])  # Creates homography from marker to camera
                # frame = draw_pyramid(frame, M_ext, Switch[ids[0, :]], ids[0, :])  # Pass in values to draw the pyramid
                cv.aruco.drawAxis(image=frame, cameraMatrix=K, distCoeffs=None, rvec=rvec_m_c, tvec=tm_c, length=MARKER_LENGTH / 2)
            
            #use M_ext to determine location of bot relative to world. 
            P_B_B = np.block([[0,0,1]]).T #may need to be 0,0,0,1
            P_b_w = np.linalg.inv(M_w_c) @ M_b_c @ P_B_B
        # Display the resulting frame
        cv.imshow('frame', frame)

        if cv.waitKey(1) == ord('q'):
            break
        cv.setMouseCallback('frame', click_event)
        left_pwm, right_pwm = 255, 255 #calculate pwms
        transmitToArduino()
    # When everything done, release the capture
    cap.release()

main()