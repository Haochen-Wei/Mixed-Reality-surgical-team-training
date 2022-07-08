#Using Aruco board for pose estimation, both background subtraction and DVRK camera locolization.
import cv2
import numpy as np

def generate_board():
    #This function using preprovided DICT_6X6_250 dictionary to create a ArUco board. Board is suitable for printout in letter size paper. Please convert to pdf first to avoid any zooming.
    x_len=4
    y_len=5
    marker_len=0.04
    marker_sep=0.01
    #Define dictionary  
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    #Generate the marker
    board = cv2.aruco.GridBoard_create(	x_len, y_len, marker_len, marker_sep, dictionary)
    
    #Two lines below for drawing board image and print it out
    board_img=cv2.aruco.drawPlanarBoard(board,(1700,2200),marginSize=102,borderBits=1)
    
    #Uncomment following line to write the board file
    cv2.imwrite("board2.png", board_img)
    
    #Create the parameter for future board detection
    parameters = cv2.aruco.DetectorParameters_create()

    return board,dictionary,parameters

# Create board object
board,dictionary,detect_parameters=generate_board()

# running camera
vid = cv2.VideoCapture(1)
key = -1

# Camera should be pre caliberated and stored here.
camera_matrix=np.array([[616.34,0,336.96],[0,615.32,244.17],[0,0,1]])
distCoeffs=np.array([[-0.1288,0.1813,0,0,0]])

# Code that will return the Rotation and translation of object relatively to the camera. unit in meter.
while key == -1:
    ret,image=vid.read()
    success=0
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=detect_parameters)
    if np.size(markerIds)>1:
        success,R,t=cv2.aruco.estimatePoseBoard(markerCorners, markerIds,board,camera_matrix,distCoeffs,None,None)
    if success!=0:
        cv2.aruco.drawDetectedMarkers(image, markerCorners)
        cv2.drawFrameAxes(image, camera_matrix, distCoeffs, R, t, 0.2)
    cv2.imshow('videocap', image)
    key=cv2.waitKey(5)
cv2.destroyAllWindows()
vid.release()