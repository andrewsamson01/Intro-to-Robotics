"""
John Ripple and Andrew Samson
MEGN 441 Lab 5
Aruco Detection
"""

import cv2 as cv  

import numpy as np
import serial
from Controls import calculatePWMS #Computes PWMS to transmit

# from Brains import transmitToArduino
# import argparse
# import imutils
# import sys

selected_coordinates = (0, 0) #globals because we need it

# Set up serial communication
ser = serial.Serial('COM13', 115200, timeout=1)
ser.reset_input_buffer()


def serialSend(PWM):
    s = " ".join(map(str, PWM)) + " \n"
    ser.write(s.encode('utf-8'))
    line = ser.readline().decode('utf-8').rstrip()
    print(line)


def click_event(event, x, y, flags, params):
    global selected_coordinates
    if event == cv.EVENT_LBUTTONDOWN:
        #was clicked
        selected_coordinates = (x, y)
        print(x, y)
        print('\n\n\n')


def getPoses():
    x, y, z, w = 0, 0, 0, 0
    return x, y, z, w


def cameraToWorld(M_ext, K):
    Point = K @ M_ext @ np.array([0, 0, 0, 1]).T
    Point = Point / Point[2]
    Z = M_ext[2][3]
    fx = K[0][0]
    fy = K[1][1]
    # print(Point)
    X = ((selected_coordinates[0] - Point[0]) * Z / fx)
    Y = (selected_coordinates[1] - Point[1]) * Z / fy * -1
    return [X, Y, Z]


def unitVecCalc(M, K, Point=np.array([1, 0, 0, 1]).T):
    camera_Point_x = K @ M @ Point
    camera_Point_x = camera_Point_x / camera_Point_x[2]
    camera_Point_origin = K @ M @ np.array([0, 0, 0, 1]).T
    camera_Point_origin = camera_Point_origin / camera_Point_origin[2]
    unit_vec = camera_Point_x - camera_Point_origin
    return unit_vec / np.linalg.norm(unit_vec)


def unitVecCalcWorld(M, K):
    camera_Point_x = [selected_coordinates[0], selected_coordinates[1], 1]
    camera_Point_origin = K @ M @ np.array([0, 0, 0, 1]).T
    camera_Point_origin = camera_Point_origin / camera_Point_origin[2]
    unit_vec = camera_Point_x - camera_Point_origin
    return unit_vec / np.linalg.norm(unit_vec)


def main():
    scale = 1
    cap = cv.VideoCapture(1, cv.CAP_DSHOW)  # this is the magic!
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920/scale)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080/scale)
    data = np.load('camera_distort_matrices.npz')
    user_world_coords = np.array([[0, 0, 0, 0]])


    # Camera parameters
    K = data['mtx']
    print(K)
    # Real world lengths in cm
    MARKER_LENGTH = 15/scale

    # Get the pattern dictionary for 7X7 markers, with ids 0 through 99.
    arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)
    
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        user_coords = np.array([[selected_coordinates[0] - 1920/2, selected_coordinates[1] -1080/2, 0, 1]]).T
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
            # print(tvecs[0])
            cv.circle(frame, (int(1920/2), int(1080/2)), 4, (255, 0, 0))
            M_ext = [None] * 2


            unit_vec_bot = []
            unit_vec_world = []
            for i in range(0, len(ids)):
                rvec_m_c = rvecs[i]  # This is a 1x3 rotation vector
                tm_c = tvecs[i]  # This is a 1x3 translation vector
                rotation_m_c = cv.Rodrigues(rvec_m_c[0])  # Gets 3x3 rotation matrix from 1x3 rotation vector
                if ids[i] == 1:
                    M_ext[1] = np.block([[rotation_m_c[0], tm_c.T]])  # Creates homography from marker to camera
                elif ids[i] == 0:
                    M_ext[0] = np.block([[rotation_m_c[0], tm_c.T]])  # Creates homography from marker to camera
                # frame = draw_pyramid(frame, M_ext, Switch[ids[0, :]], ids[0, :])  # Pass in values to draw the pyramid
                cv.aruco.drawAxis(image=frame, cameraMatrix=K, distCoeffs=None, rvec=rvec_m_c, tvec=tm_c, length=MARKER_LENGTH / 2)
                #M_w_c = np.block([[M_ext[0]], [0, 0, 0, 1]])
                M_b_c = np.block([[M_ext[1]], [0, 0, 0, 1]])
                #use M_ext to determine location of bot relative to world.
                P_B_B = np.array([[0,0,0,1]]).T
                P_B_C = K @ M_ext[1] @ P_B_B
                P_B_C = P_B_C / P_B_C[2]
                # P_b_w = np.linalg.inv(M_w_c) @ M_b_c @ P_B_B
                #user_world_coords = cameraToWorld(M_ext[0], K)
                unit_vec_bot = unitVecCalc(M_ext[1], K)
                unit_vec_world = unitVecCalcWorld(M_ext[1], K)
                angle = np.math.atan2(np.linalg.det([unit_vec_world[0:2] , unit_vec_bot[0:2]]),np.dot(unit_vec_world, unit_vec_bot))
                # print(user_world_coords)
                cv.circle(frame, (int(selected_coordinates[0]), int(selected_coordinates[1])), 4, (0, 255, 0))
                # print(angle)
                lPWM, rPWM = calculatePWMS( P_B_C[0], P_B_C[1], angle, selected_coordinates[0], selected_coordinates[1])
                serialSend([lPWM, rPWM])
                #print()
        #print(user_world_coords)
        # Display the resulting frame
        cv.imshow('frame', frame)

        if cv.waitKey(1) == ord('q'):
            break
        cv.setMouseCallback('frame', click_event)

        #left_pwm, right_pwm = calculatePWMS() #calculate pwms
        # transmitToArduino()
    # When everything done, release the capture
    cap.release()

try:
    main()
except:
    serialSend([0, 0])

