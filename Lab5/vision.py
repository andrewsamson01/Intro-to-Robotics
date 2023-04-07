"""
John Ripple and Andrew Samson
MEGN 441 Lab 5
Aruco Detection
"""

import cv2 as cv
import numpy as np
# import argparse
# import imutils
# import sys


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
        # Display the resulting frame
        cv.imshow('frame', frame)
        if cv.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    cap.release()

main()