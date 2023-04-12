import math
import numpy as np
def calculatePWMS(x,y,w, xp, yp):
    '''Takes in the x, y , and w of the robot as well as the xp and yp of the selected point
    all relative to the world returns the left PWM and right PWM'''
    angle_tolerance = math.pi / 12
    distance_tolerance = 0.5
    vector = [xp -x, yp -y]
    polar_vector = [math.sqrt(vector[0]**2 + vector[1] **2), math.atan2(vector[1]/vector[0])]
    if abs(polar_vector[1] - w) > angle_tolerance:
        angle_error = polar_vector[1] - w
        return drive(angle_error , np.sign(angle_error), -np.sign(angle_error))
    elif polar_vector[0] > distance_tolerance:
        return drive(polar_vector[0], 1, 1)
    #return leftPWM, rightPWM

def drive(distance, ldir, rdir):
    GAIN_A = 3
    GAIN_B = 3
    deadband_A = 100
    deadband_B = 100

    leftPWM = computeCommand(GAIN_A, deadband_A, distance) *  ldir
    rightPWM = computeCommand(GAIN_B, deadband_B, distance) * rdir

    return leftPWM, rightPWM

def computeCommand(gain, deadband, error):
    maxPWM = 220
    cmdDir = (gain * error) # Proportional control 
    cmdDir = max([deadband, min([cmdDir, maxPWM])]) # Bind value between motor's min and max
    return(cmdDir)
