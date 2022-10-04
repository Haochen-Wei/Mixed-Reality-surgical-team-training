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
    #cv2.imwrite("board2.png", board_img)
    
    #Create the parameter for future board detection
    parameters = cv2.aruco.DetectorParameters_create()

    return board,dictionary,parameters


def generate_marker(id,size):
    #This function using preprovided DICT_6X6_250 dictionary to create a ArUco board. Board is suitable for printout in letter size paper. Please convert to pdf first to avoid any zooming.
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    #Generate the marker
    marker = cv2.aruco.drawMarker(dictionary,id,size)
    
    #Uncomment following line to write the board file
    cv2.imwrite("marker.png", marker)

    return marker

x=generate_marker(10,100)
cv2.imshow("xx",x)
cv2.waitKey()