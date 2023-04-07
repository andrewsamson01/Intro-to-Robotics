import numpy as np
import cv2 as cv
import glob
 
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

#If these values are not the number of corners on the board then it will not work
#Where black meets white squares across and down
squarex = 6
squarey = 8

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((squarex*squarey,3), np.float32)
objp[:,:2] = np.mgrid[0:squarex,0:squarey].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
 
images = glob.glob('Calib_Pics/*.jpg')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)

    # Find the chess board corners use where the corners meet
    ret, corners = cv.findChessboardCorners(gray, (squarex,squarey),None)
     
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners)
 
        # Draw and display the corners
        cv.drawChessboardCorners(img, (squarex,squarey), corners,ret)
        cv.imshow('img',img)
        cv.waitKey(500)
    else:
        print('Not enough corners on ' + fname)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
img = cv.imread('Calib_Pics/2.jpg')
h,  w = img.shape[:2]
newcameramtx, roi=cv.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
 
np.savez('camera_distort_matrices', mtx=mtx, dist=dist, newcameramtx=newcameramtx)
# crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult.png',dst)

cv.destroyAllWindows()