"""
John Ripple and Andrew Samson
MEGN 441 Lab 5
Aruco Detection
"""

import cv2 as cv
import numpy as np
import serial
from Controls import calculatePWMS #Computes PWMS to transmit

selected_coordinates = (0, 0) #globals because we need it

# Set up serial communication
ser = serial.Serial('COM13', 115200, timeout=1)
ser.reset_input_buffer()


def serialSend(PWM):
    """Send data to Arduino Mega through serial connection"""
    s = " ".join(map(str, PWM)) + " \n"
    ser.write(s.encode('utf-8'))
    line = ser.readline().decode('utf-8').rstrip()
    print(line)


def click_event(event, x, y, flags, params):
    """Get xy pixel location the user clicked on the camera display"""
    global selected_coordinates
    if event == cv.EVENT_LBUTTONDOWN:
        #was clicked
        selected_coordinates = (x, y)
        print(x, y)
        print('\n\n\n')


def cameraToWorld(M_ext, K):
    """Go from 2D to 3D coordinates assuming Z is constant from the world Aruco marker"""
    Point = K @ M_ext @ np.array([0, 0, 0, 1]).T    # Transform the origin of the robot to camera coordinates
    Point = Point / Point[2]    # [x1, x2, x3] Need to divide x3 so the Point is [x, y, 1]
    Z = M_ext[2][3] # Get the Z coordinate from the Aruco tag
    fx = K[0][0]    # Get the camera focal lengths
    fy = K[1][1]
    # print(Point)
    X = ((selected_coordinates[0] - Point[0]) * Z / fx) # Find X and Y of the user coordinates in the world coordinates
    Y = (selected_coordinates[1] - Point[1]) * Z / fy * -1
    return [X, Y, Z]


def unitVecCalc(M, K, Point=np.array([1, 0, 0, 1]).T):
    """Get a unit vector from the camera origin along the x axis and the camera origin to the user point"""
    camera_Point_x = K @ M @ Point
    camera_Point_x = camera_Point_x / camera_Point_x[2]
    camera_Point_origin = K @ M @ np.array([0, 0, 0, 1]).T
    camera_Point_origin = camera_Point_origin / camera_Point_origin[2]
    unit_vec = camera_Point_x - camera_Point_origin
    return unit_vec / np.linalg.norm(unit_vec)


def unitVecCalcWorld(M, K):
    """Get a unit vector from the camera origin along the x axis and the camera origin to the user point"""
    camera_Point_x = [selected_coordinates[0], selected_coordinates[1], 1]
    camera_Point_origin = K @ M @ np.array([0, 0, 0, 1]).T
    camera_Point_origin = camera_Point_origin / camera_Point_origin[2]
    unit_vec = camera_Point_x - camera_Point_origin
    return unit_vec / np.linalg.norm(unit_vec)


def main():
    """Main function to create camera feed and allow user input to direct the robot to a specific location"""
    # Set up the video stream
    scale = 1
    cap = cv.VideoCapture(1, cv.CAP_DSHOW)  # this is the magic!
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920/scale)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080/scale)
    data = np.load('camera_distort_matrices.npz')   # Load the camera intrinsic parameters
    user_world_coords = np.array([[0, 0, 0, 0]])


    # Camera parameters
    K = data['mtx']
    print(K)
    # Real world lengths in cm (the Aruco tag was printed to be 15 x 15 cm)
    MARKER_LENGTH = 15/scale

    # Get the pattern dictionary for 7X7 markers, with ids 0 through 99.
    arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)
    
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        user_coords = np.array([[selected_coordinates[0] - 1920/2, selected_coordinates[1] -1080/2, 0, 1]]).T   # Continually update user coordinates
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Detect any aruco markers in the image
        corners, ids, _ = cv.aruco.detectMarkers(image=frame, dictionary=arucoDict)
        # Only do matrix calculations if there is an Aruco Marker in the image
        if ids is not None:
            cv.aruco.drawDetectedMarkers(image=frame, corners=corners, ids=ids, borderColor=(0, 0, 255))
            # Get the pose of all markers in the image
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners=corners, markerLength=MARKER_LENGTH, cameraMatrix=K, distCoeffs=None)
            # Get the pose of the first detected marker with respect to the camera.
            # print(tvecs[0])
            cv.circle(frame, (int(1920/2), int(1080/2)), 4, (255, 0, 0))
            M_ext = [None] * 2

            # Draw Aruco Marker axis for every marker in the frame
            unit_vec_bot = []
            unit_vec_world = []
            for i in range(0, len(ids)):
<<<<<<< Updated upstream
                try:
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
                except:
                    continue
=======
                rvec_m_c = rvecs[i]  # This is a 1x3 rotation vector
                tm_c = tvecs[i]  # This is a 1x3 translation vector
                rotation_m_c = cv.Rodrigues(rvec_m_c[0])  # Gets 3x3 rotation matrix from 1x3 rotation vector
                if ids[i] == 1:
                    M_ext[1] = np.block([[rotation_m_c[0], tm_c.T]])  # Creates homography from marker to camera
                elif ids[i] == 0:
                    M_ext[0] = np.block([[rotation_m_c[0], tm_c.T]])  # Creates homography from marker to camera
                # Draw axis of Aruco Marker
                cv.aruco.drawAxis(image=frame, cameraMatrix=K, distCoeffs=None, rvec=rvec_m_c, tvec=tm_c, length=MARKER_LENGTH / 2)
                #M_w_c = np.block([[M_ext[0]], [0, 0, 0, 1]])
                M_b_c = np.block([[M_ext[1]], [0, 0, 0, 1]])    # Create homography matrix (4x4) for the bot to camera
                #use M_ext to determine location of bot relative to world.
                P_B_B = np.array([[0,0,0,1]]).T     # Position of the bot origin
                P_B_C = K @ M_ext[1] @ P_B_B        # Position of bot in camera coordinates
                P_B_C = P_B_C / P_B_C[2]            # Divide by third matrix position to get camera coordinates
                # P_b_w = np.linalg.inv(M_w_c) @ M_b_c @ P_B_B
                #user_world_coords = cameraToWorld(M_ext[0], K)
                unit_vec_bot = unitVecCalc(M_ext[1], K)     # Calculate the unit vector to the bot
                unit_vec_world = unitVecCalcWorld(M_ext[1], K)  # Calculate the unit vector to the user coordinate
                angle = np.math.atan2(np.linalg.det([unit_vec_world[0:2] , unit_vec_bot[0:2]]),np.dot(unit_vec_world, unit_vec_bot))    # Find the angle (angle error for p controller) between the two previously calculated unit vectors
                # print(user_world_coords)
                cv.circle(frame, (int(selected_coordinates[0]), int(selected_coordinates[1])), 4, (0, 255, 0))  # Draw user coordinate onto the stream
                # print(angle)
                lPWM, rPWM = calculatePWMS( P_B_C[0], P_B_C[1], angle, selected_coordinates[0], selected_coordinates[1])    # Calculate the pwm to send to the bot
                serialSend([lPWM, rPWM])    # Send the PWMs to the transmitter Arduino Mega over serial
>>>>>>> Stashed changes
                #print()
        #print(user_world_coords)
        # Display the resulting frame
        cv.imshow('frame', frame)

        if cv.waitKey(1) == ord('q'):
            break
        cv.setMouseCallback('frame', click_event)
    # When everything done, release the capture
    cap.release()


try:
    main()
<<<<<<< Updated upstream
except Exception as e:
    print(e)
=======
except:
    # If there is an error, tell the robot to stop moving
>>>>>>> Stashed changes
    serialSend([0, 0])

