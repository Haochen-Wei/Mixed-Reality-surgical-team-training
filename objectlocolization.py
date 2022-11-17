import cv2
import numpy as np
import native
import json

def generate_marker(id,size):
    #This function using preprovided DICT_6X6_250 dictionary to create a ArUco board. Board is suitable for printout in letter size paper. Please convert to pdf first to avoid any zooming.
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    #Generate the marker
    marker = cv2.aruco.drawMarker(dictionary,id,size)
    
    #Uncomment following line to write the board file
    cv2.imwrite("marker.png", marker)

    return marker

class Resolution :
   # Laptop
    width =  1280 
    height = 720
    
#Open camera
cap = cv2.VideoCapture("/dev/video2")
if cap.isOpened() == 0:
    print("Can not open Desiginated camera")
    exit(-1)
image_size = Resolution()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width*2)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)

#Read configuration and find camera matrix
calibration_file = "SN10028124.conf"
#P1 for left camera, P2 for right camera
P1, P2, map_left_x, map_left_y, map_right_x, map_right_y,K_left,dist_left = native.init_calibration(calibration_file, image_size)

#Define the parameter for the marker
parameters = cv2.aruco.DetectorParameters_create()
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)


key=''
tvec1=np.zeros(3)
tvec2=np.zeros(3)
tvec3=np.zeros(3)
tvec4=np.zeros(3)

while key != 113:
    retval, frame = cap.read()
    # Extract left and right images from side-by-side
    img = np.split(frame, 2, axis=1)[0]
    cv2.imshow("video",img)
    markerCorners, markerIds,_=cv2.aruco.detectMarkers(img, dictionary,parameters=parameters) 
    #Size is the physical size of marker in mm
    size=25
    if markerIds!=None:
        rvecs, tvecs,_=cv2.aruco.estimatePoseSingleMarkers(markerCorners, size, K_left, dist_left)
        tvec4=tvec3
        tvec3=tvec2
        tvec2=tvec1
        tvec1=tvecs[0]
        rvec=rvecs[0][0]
    
    key=cv2.waitKey(50)


result=(tvec1+tvec2+tvec3+tvec4)/4
print(result)
dic={"x":result[0][0],"y":result[0][1],"z":result[0][2],"r1":rvec[0],"r2":rvec[1],"r3":rvec[2]}

with open("position.json","w") as f:
    json.dump(dic,f)


# f=open("position.json")
# dic=json.load(f)
# x_obj=dic['x']
# y_obj=dic['y']
# z_obj=dic['z']
# f.close()
