#Using Aruco board for pose estimation, both background subtraction and DVRK camera locolization.
import cv2
import numpy as np


#Same as aruco_board file, but use checoboard.
def generate_board():
    #This function using preprovided DICT_6X6_250 dictionary to create a ArUco board. Board is suitable for printout in letter size paper. Please convert to pdf first to avoid any zooming.
    x_len=7
    y_len=9
    marker_len=0.03
    marker_sep=0.02
    #Define dictionary  
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    #Generate the marker
    board = cv2.aruco.CharucoBoard_create( x_len, y_len, marker_len, marker_sep, dictionary)
    
    #Two lines below for drawing board image and print it out
    board_img=cv2.aruco.CharucoBoard.draw(board,(1700,2200),marginSize=23,borderBits=1)
    cv2.imwrite("board.png", board_img)
    
    #Create the parameter for future board detection
    parameters = cv2.aruco.DetectorParameters_create()

    return board,dictionary,parameters

#Following function is used to detect the pose of board, simply pack the cv functions
def detect_pose(image,board,dictionary,parameters,camera_matrix,distCoeffs):
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
    charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(markerCorners,markerIds,image,board)
    if np.size(charucoIds)>0:
        R,t=cv2.aruco.estimatePoseCharucoBoard(charucoCorners, markerIds,board,camera_matrix,distCoeffs)
        return R,t
    return 0,0

# Use following line to write the board file in current 
board,dictionary,detect_parameters=generate_board()
vid = cv2.VideoCapture(1)
key = -1
CamMat=np.array([[274.1922594,0,284.18394809],[0,62.97150065,226.07369862],[0,0,1]])
Dist_Cof=np.array([-3.94404638e-02,2.02725037e-04,2.60642096e-02,-3.03679258e-03,5.22064531e-07])
while key == -1:
    ret,image=vid.read()
    R,t=detect_pose(image,board,dictionary,detect_parameters,CamMat,Dist_Cof)
    if R!=0:
        cv2.drawFrameAxes(	image, CamMat, Dist_Cof, R, t, 10)
cv2.destroyAllWindows()
vid.release()