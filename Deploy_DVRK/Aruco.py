#Using Aruco board for pose estimation, both background subtraction and DVRK camera locolization.
import cv2
import numpy as np

def generate_board(x_len=5,y_len=7,marker_len=0.04,marker_sep=0.01): #This function using preprovided DICT_6X6_250 dictionary to create a ArUco board

    #Define dictionary  
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    #Generate the marker
    board = cv2.aruco.GridBoard_create(	x_len, y_len, marker_len, marker_sep, dictionary)
    
    #Two lines below for drawing board image and print it out
    board_img=cv2.aruco.drawPlanarBoard(board,(1000,500),marginSize=50,borderBits=1)
    cv2.imwrite("board.png", board_img)
    
    #Create the parameter once for reuse in the future
    parameters = cv2.aruco.DetectorParameters_create()

    return board,dictionary,parameters

# Use following line to print the board
#board,dictionary,detect_parameters=generate_board(5,7,0.04,0.01)

#Following function is used to detect the pose of board, simply pack the cv functions
def detect_pose(image,board,dictionary,parameters,camera_matrix):
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
    distCoeffs=np.array([[0], [0], [0], [0], [0]])
    R,t=cv2.aruco.estimatePoseBoard(markerCorners, markerIds,board,camera_matrix,distCoeffs)
    return R,t