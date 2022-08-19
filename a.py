import cv2
import recon_point as rp
import numpy as np
import board_detection as bd
import native
import time

# backSub = cv2.createBackgroundSubtractorKNN()
# backSub.setHistory(5000)
# backSub.setShadowValue(255)

board,dictionary,detect_parameters=bd.generate_board()
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
class Resolution :
   # Laptop
    width =  1280
    height = 720
image_size = Resolution()

#Laptop
# image_size.width = 672 
# image_size.height = 376

cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width*2)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)

end_cap = cv2.VideoCapture(2,cv2.CAP_DSHOW)
camera_matrix_end=np.array([[616.34,0,336.96],[0,615.32,244.17],[0,0,1]])
distCoeffs_end=np.array([[-0.1288,0.1813,0,0,0]])

calibration_file = "SN10028124.conf"
#P1 for left camera, P2 for right camera
P1, P2, map_left_x, map_left_y, map_right_x, map_right_y,K_left,dist_left = native.init_calibration(calibration_file, image_size)


while True:
    X=time.time()
    ret,image_end=end_cap.read()
    ret,image=cap.read()
    left_right_image = np.split(image, 2, axis=1)
    left_rect = cv2.remap(left_right_image[0], map_left_x, map_left_y, interpolation=cv2.INTER_LINEAR)


    success_end=0
    markerCorners_end, markerIds_end, _ = cv2.aruco.detectMarkers(image_end, dictionary, parameters=detect_parameters)
    if np.size(markerIds_end)>1:
        success_end,R_end,t_end=cv2.aruco.estimatePoseBoard(markerCorners_end, markerIds_end,board,camera_matrix_end,distCoeffs_end,None,None)
        # cv2.drawFrameAxes(image_end, camera_matrix_end, distCoeffs_end, R_end, t_end, 0.05)
    
    success_main=0
    markerCorners_main, markerIds_main, _ = cv2.aruco.detectMarkers(left_right_image[0], dictionary, parameters=detect_parameters)
    if np.size(markerIds_main)>1:
        success_main,R_main,t_main=cv2.aruco.estimatePoseBoard(markerCorners_main, markerIds_main,board,K_left,dist_left,None,None)
        # cv2.drawFrameAxes(left_rect, K_left, dist_left, R_main, t_main, 0.05)
    
    cv2.imshow("Test",left_rect) 
    
    # cv2.imshow("Test_end",image_end)


    if success_main!=0 and success_end!=0:
        # Construct and invert the E_end
        rot_end,_=cv2.Rodrigues(R_end)
        E_end=np.hstack((rot_end.T,np.matmul(-1*rot_end.T,t_end)))
        E_end=np.vstack((E_end,np.array([[0.,0.,0.,1.]])))
        # Construct the E_main
        rot_main,_=cv2.Rodrigues(R_main)
        E_main=np.hstack((rot_main,t_main))
        E_main=np.vstack((E_main,np.array([[0.,0.,0.,1.]])))
        E=np.matmul(E_main,E_end)

    else:
        E=np.eye(4,dtype=np.float32)
  
    # g = backSub.apply(img)
    R=cv2.Rodrigues(E[0:3,0:3])[0]
    cv2.drawFrameAxes(left_rect, K_left, dist_left, R, E[0:3,3], 0.05)
    cv2.imshow("Test",left_rect)
    key=cv2.waitKey(5)
    print("time is",1/(time.time()-X))
    if key!=-1:
        break


