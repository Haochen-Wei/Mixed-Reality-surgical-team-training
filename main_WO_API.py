#This function combined the ZED mini API with the centerline detection.
import cv2
import filter
import recon_point as rp
import numpy as np
import time
import zmq
import native
import Registeration
#=====================================================================================================================================================
#Initial Setup
#Create a ZMQ socket
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

#Set resolution
class Resolution :
    width = 1280 #672 for laptop
    height = 720 #376 for laptop

#Open camera
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
if cap.isOpened() == 0:
    print("Can not open Desiginated camera")
    exit(-1)
image_size = Resolution()
image_size.width = 1280 #672
image_size.height = 720 #376

# Set the video resolution to HD720 (VGA for laptop)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_size.width*2)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size.height)

# Read configuration and find camera matrix
calibration_file = "SN10028124.conf"
camera_matrix_left, camera_matrix_right, map_left_x, map_left_y, map_right_x, map_right_y = native.init_calibration(calibration_file, image_size)
P1=camera_matrix_left
P2=camera_matrix_right

#Create the bg class, using kNN method, reference opencv background subtraction.
#backSub = cv.createBackgroundSubtractorMOG2()
backSub_left = cv2.createBackgroundSubtractorKNN()
backSub_right = cv2.createBackgroundSubtractorKNN()

#=====================================================================================================================================================
#Following block is used to registe all the equipments Modify this part to debug
#Define how many tools will be used first.
number_flag=0
while number_flag==0:
    try:
        tool_count=int(input("Please enter the number of tools you want to use (maximum 8):"))
        if tool_count<=0 or tool_count>8: 
            print("Number out of range")
        else:
            number_flag=1
    except ValueError:
        print("Please enter a number")

#Two list used for store the status for future calculating the piviot points.
fixed_left_list=[]
fixed_right_list=[]

for i in range(tool_count):
#Start the registeration process
    input("Please insert the "+str(i+1)+"th tool into the first entry port then press any bottom to continue")
    print("Please move around the tool, once finished, press q to continue")
    key = ''
    line_left_list=[]
    line_right_list=[]
    j=0
    while key != 113:  # for 'q' key
        j+=1
        start=time.time()
        retval, frame = cap.read()
        # Extract left and right images from side-by-side
        left_right_image = np.split(frame, 2, axis=1)
        left_rect = cv2.remap(left_right_image[0], map_left_x, map_left_y, interpolation=cv2.INTER_LINEAR)
        right_rect = cv2.remap(left_right_image[1], map_right_x, map_right_y, interpolation=cv2.INTER_LINEAR)

        #Old binary method
        '''g_l=cv2.cvtColor(left_rect,cv2.COLOR_RGB2GRAY)
        g_r=cv2.cvtColor(right_rect,cv2.COLOR_RGB2GRAY)
        max_l=np.amax(g_l)
        max_r=np.amax(g_r)
        g_l=np.uint8(g_l*(255/max_l))
        g_r=np.uint8(g_r*(255/max_r))
        cv2.imshow("l",g_l)
        [_,Binary_l]=cv2.threshold(g_l,70,255,cv2.THRESH_BINARY)
        [_,Binary_r]=cv2.threshold(g_r,70,255,cv2.THRESH_BINARY)'''

        #Background subtraction
        g_l = backSub_left.apply(left_rect)
        g_r = backSub_right.apply(right_rect)

        cv2.imshow("left_diff",g_l)
        cv2.imshow("right_diff",g_r)

        # Center_line left graph
        mid_list_l=filter.centerline(g_l)
        if len(mid_list_l)!=0:
            line_left,_=filter.get_line(g_l,mid_list_l,[])
            for i in range(len(line_left)):
                line_left_list.append(line_left[i])#g_l replaced Binary_l
        if len(line_left_list)>100: #Limit the lenth of list to 100 to aviod memory explode
            line_left_list.pop(0)

        # Center_line right graph
        mid_list_r=filter.centerline(g_r)
        if len(mid_list_r)!=0:
            line_right,_=filter.get_line(g_r,mid_list_r,[])
            for i in range(len(line_right)):
                line_right_list.append(line_right[i])#g_r replaced Binary_r
        if len(line_right_list)>100: #Limit the lenth of list to 100 to aviod memory explode
            line_right_list.pop(0)
        key = cv2.waitKey(5)
    
    cap.release()
    cv2.destroyAllWindows()
    fixed_left_list.append(Registeration.reg_2d(line_left_list))
    fixed_right_list.append(Registeration.reg_2d(line_right_list))


print("left",fixed_left_list)
print("right",fixed_right_list)

exit(-1)

#=====================================================================================================================================================
#Start the main function
key = ''
while key != 113:  # for 'q' key
    start=time.time()
    retval, frame = cap.read()
    # Extract left and right images from side-by-side
    left_right_image = np.split(frame, 2, axis=1)
    left_rect = cv2.remap(left_right_image[0], map_left_x, map_left_y, interpolation=cv2.INTER_LINEAR)
    right_rect = cv2.remap(left_right_image[1], map_right_x, map_right_y, interpolation=cv2.INTER_LINEAR)
    
    '''
    Need be filling out for recitify the movement using new_R and new_t, test no recitify first
    new_R,new_t=Aruco.detect_pose(left_right_image[0],board,dictionary,detect_parameters,P1)
    '''

    #Following is the old binary method
    '''g_l=cv2.cvtColor(left_rect,cv2.COLOR_RGB2GRAY)
    g_r=cv2.cvtColor(right_rect,cv2.COLOR_RGB2GRAY)
    max_l=np.amax(g_l)
    max_r=np.amax(g_r)
    g_l=np.uint8(g_l*(255/max_l))
    g_r=np.uint8(g_r*(255/max_r))
    cv2.imshow("l",g_l)
    [_,Binary_l]=cv2.threshold(g_l,70,255,cv2.THRESH_BINARY)
    [_,Binary_r]=cv2.threshold(g_r,70,255,cv2.THRESH_BINARY)'''

#####Need to filling Background sub here

    #left graph
    mid_list_l=filter.centerline(g_l)
    line_l,index_l=filter.get_line(g_l,mid_list_l,fixed_left_list)
    left_point=filter.classify_point(mid_list_l,line_l)

    #right graph
    mid_list_r=filter.centerline(g_r)
    line_r,index_r=filter.get_line(g_r,mid_list_r,fixed_right_list)
    right_point=filter.classify_point(mid_list_r,line_r)
    

    #================================================================================================================
    #This block is used to show the left and right filtered points   
    #Show the line point
    for i in range(len(left_point)):
        for j in range(len(left_point[i])):
            cv2.circle(left_rect,(left_point[i][j][0],left_point[i][j][1]),1,(0,255,0))
    cv2.imshow("left_line",left_rect)
    #Show the line point
    for i in range(len(right_point)):
        for j in range(len(right_point[i])):
            cv2.circle(right_rect,(right_point[i][j][0],right_point[i][j][1]),1,(0,255,0))
    cv2.imshow("right_line",right_rect)
    #=================================================================================================================
    #This code block is for the 3d reconstruction process
    a3dline=[]
    for i in range (len(index_l)):
        for j in range(len(index_r)):
            #if abs(left_point[i][-1][1]-right_point[j][-1][1])<5: #and abs(len(left_point)-len(right_point))<30: # This step to find the same line in stereo image.
            if index_l[i]==index_r[j]:
                #Triangulation find the point
                a3dpoint=rp.point_cloud(P1,P2,left_point[i],right_point[j])
                object_point=a3dpoint[0:3,:]
                
                #Find the equation of 3d line using least square
                xs = a3dpoint[0,:]
                ys = -1*a3dpoint[1,:]
                zs = a3dpoint[2,:]

                ones_y=np.ones(np.size(ys))
                ys=np.vstack((ys,ones_y)).T

                result_x=np.linalg.lstsq(ys,xs,rcond=None)
                xk=result_x[0][0]
                xb=result_x[0][1]
                result_z=np.linalg.lstsq(ys,zs,rcond=None)
                zk=result_z[0][0]
                zb=result_z[0][1]
                a3dline.append([xk,xb,zk,zb,min(ys[:,0])])
    

    #=======================================================================================================================
    #Send the data of line via ZMQ
    data=""
    topic='tool'
    for i in range(len(a3dline)):
        for j in range(5):
            data=data+str(a3dline[i][j])+"_"
        data=data+'!'
    if len(a3dline)>0:
        msg = [topic.encode("utf-8"), data.encode("utf-8")]
        socket.send_multipart(msg)
        print(msg)
    end=time.time()
    # Print the FPS
    if end-start!=0:
        print('FPS is ',1/(end-start))
    key = cv2.waitKey(5)

#=====================================================================================================================================
#Main function end here
#Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()