'''
This program takes a user input filename and saves a picture to it.
The picture can be resized to half its size and turned grayscale.
'''

import cv2 as cv
import numpy as np
import time

clr = True




#Sets up camera
camera = cv.VideoCapture(0, cv.CAP_DSHOW)  # this is the magic!
camera.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

cv.namedWindow('image')
print("t takes a picture, s saves, m changes colorspace, r retakes picture")
i = 0
while True:
    ret, image = camera.read()
    #Creates a window that can be used with a mouse
    cv.imshow('image', image)
    #Checks what the keyboard presses are while in the window
    k = cv.waitKey(1) & 0xFF
    if k == ord('q'):
        cv.destroyAllWindows()
        exit(0)
    elif k == ord('s'):
        userInput = input("Enter a filename: ")
        userInput = userInput + ".jpg"
        cv.imwrite(userInput, image)
    elif k == ord('m'):
        clr = not clr
    elif k == ord('t'):
        userInput = "Calib_Pics\\" + str(i) + ".jpg"
        cv.imwrite(userInput, image)
        i += 1
        # while(1):
        #     k = cv.waitKey(1) & 0xFF
        #     if k == ord('r'):
        #         break
        #     elif k == ord('s'):
        #         userInput = input("Enter a filename: ")
        #         userInput = "Calib_Pics\\" + userInput + ".jpg"
        #         cv.imwrite(userInput, image)
        #         break
        #     elif k == ord('q'):
        #         cv.destroyAllWindows()
        #         exit(0)
            
camera.release()
