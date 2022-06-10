#Currently using SGBM method provided in OpenCV libeary for matching, looking to test the result, if not good enough, considering rewrite own SGM algo using C++
import cv2 
import numpy as np
def tool_pointcloud(img1,img2,Q):
    blockSize=3
    stereo=cv2.StereoBM_create(	minDisparity=0,
        numDisparities=16,
        blockSize=blockSize,
        P1=8*blockSize**2,
        P2=32*blockSize**2,
        disp12MaxDiff=5,
        uniquenessRatio=10,
        speckleWindowSize=50,
        speckleRange=1)
    #Reference the opencv file https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html to see the parameters
    disparity = stereo.compute(img1, img2) # Calculate Parallax
    disparity = np.float32(disparity)/16
    pointcloud = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=False)
    return pointcloud