import numpy as np
import cv2 as cv
import time

#Camera caliberation file, though not working very great, recommend to using matlab caliberation toolbox.

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((8*6,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:8].T.reshape(-1,2)
size=0.0235
objp=objp*size
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
vid = cv.VideoCapture(1)

key = -1

while key == -1:
    start=time.time()
    ret,img=vid.read()
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (8,6), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (8,6), corners2, ret)
        time.sleep(1)
    cv.imshow('videocap', img)
    print(time.time()-start)
    key=cv.waitKey(50)
cv.destroyAllWindows()
vid.release()


if objpoints!=[]:
    if len(objpoints)>30:
        objpoints=objpoints[-30:]
        imgpoints=imgpoints[-30:]
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    else:
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("Camera matrix",mtx)
    print("Distortion factor",dist)
