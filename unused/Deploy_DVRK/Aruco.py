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
    cv2.imwrite("board2.png", board_img)
    
    #Create the parameter for future board detection
    parameters = cv2.aruco.DetectorParameters_create()

    return board,dictionary,parameters

# Use following line to write the board file in current 
board,dictionary,detect_parameters=generate_board()

#Following function is used to detect the pose of board, simply pack the cv functions
def detect_pose(image,board,dictionary,parameters,camera_matrix):
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
    distCoeffs=np.array([[0], [0], [0], [0], [0]])
    R,t=cv2.aruco.estimatePoseBoard(markerCorners, markerIds,board,camera_matrix,distCoeffs)
    return R,t

