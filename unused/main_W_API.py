#This function combined the ZED mini API with the centerline detection.

from asyncio.windows_events import NULL
import pyzed.sl as sl
import cv2
import filter
import unused.crop as crop
import recon_point as rp
import numpy as np
from matplotlib import pyplot as plt
import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")
topic = ["t1","t2","t3","t4","t5","t6","t7","t8"]


# Create a ZED camera object
zed = sl.Camera()
# Set configuration parameters
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 30
init_params.coordinate_units = sl.UNIT.MILLIMETER

# Open the camera,Get the Projection matrix and obtain the img

status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()
runtime = sl.RuntimeParameters()
img = sl.Mat()

P1,P2,K=rp.get_P_matrix(zed)


#Single stereo image testing
#Start the showing
graph=cv2.imread("./TEST_IMG/T3.png")

#Convert to Binary
gray=cv2.cvtColor(graph,cv2.COLOR_RGB2GRAY)
[_,Binary]=cv2.threshold(gray,70,255,cv2.THRESH_BINARY)
[Binary_l,Binary_r]=crop.crop(Binary)
[g_l,g_r]=crop.crop(graph)


mid_list_l=filter.centerline(Binary_l)
line_l=filter.get_line(Binary_l,mid_list_l)
left_point=filter.classify_point(mid_list_l,line_l)



mid_list_r=filter.centerline(Binary_r)
line_r=filter.get_line(Binary_r,mid_list_r)
right_point=filter.classify_point(mid_list_r,line_r)



#Show the raw
for i in range(len(mid_list_l)):
    for j in range(len(mid_list_l[i])):
        cv2.circle(g_l,(mid_list_l[i][j],i),1,(255,0,0))
cv2.imshow("left_raw",g_l)
#Show the raw
for i in range(len(mid_list_r)):
    for j in range(len(mid_list_r[i])):
        cv2.circle(g_r,(mid_list_r[i][j],i),1,(255,0,0))
cv2.imshow("right_raw",g_r)

#Show the line point
for i in range(len(left_point)):
    for j in range(len(left_point[i])):
        cv2.circle(g_l,(left_point[i][j][0],left_point[i][j][1]),1,(0,255,0))
cv2.imshow("left_line",g_l)

#Show the line point
for i in range(len(right_point)):
    for j in range(len(right_point[i])):
        cv2.circle(g_r,(right_point[i][j][0],right_point[i][j][1]),1,(0,255,0))
cv2.imshow("right_line",g_r)
#cv2.waitKey()


plt.figure(1)
ax = plt.axes(projection="3d")
ax.scatter(0,0,0)
a3dline=[]
flag=0
for i in range(len(left_point)):
    for j in range(len(right_point)):
        if abs(left_point[i][-1][1]-right_point[j][-1][1])<10: # This step to find the same line in stereo image.
            a3dpoint=rp.point_cloud(P1,P2,left_point[i],right_point[j])
            object_point=a3dpoint[0:3,:]
            
            #Reproject back to image show the result
            re_point=cv2.projectPoints(object_point,np.zeros((3,1),dtype=np.float64),np.zeros((3,1),dtype=np.float64),K,NULL)
            a2d=re_point[0]
            for n in range(np.shape(a2d)[0]):
                cv2.circle(g_l,(int(a2d[n][0][0]),int(a2d[n][0][1])),1,(0,0,255))
            cv2.imshow("reproject",g_l)
            #cv2.waitKey()
            
            # plot 3D points
            xs = a3dpoint[0,:]
            ys = -1*a3dpoint[1,:]
            zs = a3dpoint[2,:]
            ax.scatter(xs,ys,zs)
            
            #Find the equation of 3d line
            
            ones_y=np.ones(np.size(ys))
            ys=np.vstack((ys,ones_y)).T

            # print("XS=",xs)
            # print("YS=",ys)

            result_x=np.linalg.lstsq(ys,xs,rcond=None)
            xk=result_x[0][0]
            xb=result_x[0][1]
            result_z=np.linalg.lstsq(ys,zs,rcond=None)
            zk=result_z[0][0]
            zb=result_z[0][1]
            a3dline.append([xk,xb,zk,zb,min(ys[:,0])])
            flag=1
    #         break
    # if flag==1:
    #     break
print(a3dline)

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
zed.close()
plt.show()
#End single image test here



'''
print("Start")
key = ''
while key != 113:  # for 'q' key
    err = zed.grab(runtime)
    if err == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(img, sl.VIEW.SIDE_BY_SIDE)
        start=time.time()
        #graph is the img data of this frame
        graph=img.get_data()

        #Convert to Binary
        gray=cv2.cvtColor(graph,cv2.COLOR_RGB2GRAY)
        [_,Binary]=cv2.threshold(gray,70,255,cv2.THRESH_BINARY)

        #crop graph to 2
        [Binary_l,Binary_r]=crop.crop(Binary)
        [g_l,g_r]=crop.crop(graph)

        #left graph
        mid_list_l=filter.centerline(Binary_l)
        line_l=filter.get_line(Binary_l,mid_list_l)
        left_point=filter.classify_point(mid_list_l,line_l)

        #right graph
        mid_list_r=filter.centerline(Binary_r)
        line_r=filter.get_line(Binary_r,mid_list_r)
        right_point=filter.classify_point(mid_list_r,line_r)
        
        #Show the line point
        for i in range(len(left_point)):
            for j in range(len(left_point[i])):
                cv2.circle(g_l,(left_point[i][j][0],left_point[i][j][1]),1,(0,255,0))
        cv2.imshow("left_line",g_l)


        #Show the line point
        for i in range(len(right_point)):
            for j in range(len(right_point[i])):
                cv2.circle(g_r,(right_point[i][j][0],right_point[i][j][1]),1,(0,255,0))
        cv2.imshow("right_line",g_r)

        # Following is to have the 3d reconstruction process
        a3dline=[]
        for i in range (len(left_point)):
            for j in range(len(right_point)):
                if abs(left_point[i][-1][1]-right_point[j][-1][1])<10: # This step to find the same line in stereo image.
                    a3dpoint=rp.point_cloud(P1,P2,left_point[j],right_point[i])
                    object_point=a3dpoint[0:3,:]
                    #Find the equation of 3d line
                    
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
        for i in range(len(a3dline)):
            t=topic[i]
            data=""
            for j in range(5):
                data=data+str(a3dline[i][j])+"_"
            msg = [t.encode("utf-8"), data.encode("utf-8")]
            socket.send_multipart(msg)
            print(msg)
        end=time.time()
        #if end-start!=0:
            #print('FPS is ',1/(end-start))
        key = cv2.waitKey(5)
    else:
        key = cv2.waitKey(5)
cv2.destroyAllWindows()
zed.close()'''