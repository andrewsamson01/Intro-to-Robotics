import math
import numpy as np
def calculatePWMS(x, y, w, xp, yp):
    '''Takes in the x, y , and w of the robot as well as the xp and yp of the selected point
    all relative to the world returns the left PWM and right PWM'''
    angle_tolerance = math.pi / 10
    distance_tolerance = 10
    vector = [xp - x, yp - y]
    # polar_vector = [math.sqrt(vector[0]**2 + vector[1] **2), math.atan2(vector[1], vector[0])]
    distance = math.sqrt(vector[0]**2 + vector[1] **2)
    print(distance)
    if distance < distance_tolerance: # If within the distance tolerance it is good enough
        return 0, 0
    if abs(w) > angle_tolerance: #Turn towards the point
        angle_error = w
        return drive(angle_error*10 , np.sign(angle_error), np.sign(angle_error))
    elif distance > distance_tolerance:#Drive towards the point
        return drive(distance, 1, -1)
    return 0, 0

def drive(distance, ldir, rdir):
    #A place that allows for the gain and deadband of each motor to be changed if needed. Also Determines direction of motors
    GAIN_A = 3
    GAIN_B = 3
    deadband_A = 100
    deadband_B = 100

    leftPWM = computeCommand(GAIN_A, deadband_A, distance) * ldir
    rightPWM = computeCommand(GAIN_B, deadband_B, distance) * rdir

    return leftPWM, rightPWM

def computeCommand(gain, deadband, error):
    #P Controler taht determines the PWM that based on the gain, error and deadband
    maxPWM = 220
    cmdDir = (gain * error) # Proportional control 
    cmdDir = max([deadband, min([cmdDir, maxPWM])]) # Bind value between motor's min and max
    return(cmdDir)
