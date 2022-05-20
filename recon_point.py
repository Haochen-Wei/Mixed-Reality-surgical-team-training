import cv2
import numpy as np

# obtain the projection matrixes.

def get_P_matrix(zed):

    calibration_params = zed.get_camera_information().calibration_parameters
    # Focal length of the left eye in pixels
    focal_left_x = calibration_params.left_cam.fx
    focal_left_y = calibration_params.left_cam.fy
    # Focal length of the right eye in pixels
    focal_right_x = calibration_params.right_cam.fx
    focal_right_y = calibration_params.right_cam.fy

    # cx and cy in left camera
    cx_left=calibration_params.left_cam.cx
    cy_left=calibration_params.left_cam.cy
    
    # cx and cy in right camera
    cx_right=calibration_params.right_cam.cx
    cy_right=calibration_params.right_cam.cy

    #Internel para matrix
    K_left=np.array([[focal_left_x,0,cx_left],[0,focal_left_y,cy_left],[0,0,1]])
    K_right=np.array([[focal_right_x,0,cx_right],[0,focal_right_y,cy_right],[0,0,1]])

    #Translation and rotation
    t= np.reshape(calibration_params.T,(3,1))*-1
    R,J= cv2.Rodrigues(calibration_params.R)
    
    #Projection matrix
    P1=np.matmul(K_right,np.hstack((np.eye(3),np.zeros((3,1)))))
    P2=np.matmul(K_left,np.hstack((R,t)))

    return P1,P2,K_left

def point_cloud(P1,P2,point_img1,point_img2):
    
    left=np.array(point_img1,dtype=np.float32)
    left=np.transpose(left)
    right=np.array(point_img2,dtype=np.float32)
    right=np.transpose(right)

    #Using epipolar constrain to filter the point again
    left_index=left[1,:]
    right_index=right[1,:]
    left_length=len(left_index)
    right_length=len(right_index)


    i=0
    j=0
    left_data=[]
    right_data=[]
    index=[]

    if left_length<=right_length:
        while i < left_length:
            if j>=right_length:
                break
            if left_index[i]==right_index[j]:
                left_data.append(left[0][i])
                right_data.append(right[0][j])
                index.append(left_index[i])
                i+=1
                j+=1
            elif left_index[i]>right_index[j]:
                j+=1
            elif left_index[i]<right_index[j]:
                i+=1


    if left_length>right_length:
        while j < right_length:
            if i>=left_length:
                break
            if left_index[i]==right_index[j]:
                left_data.append(left[0][i])
                right_data.append(right[0][j])
                index.append(left_index[i])
                i+=1
                j+=1
            elif left_index[i]>right_index[j]:
                j+=1
            elif left_index[i]<right_index[j]:
                i+=1
    
    # Convert the data and index to ndarray
    left_np=np.array(left_data)
    right_np=np.array(right_data)
    index_np=np.array(index)
    left_p=np.vstack((left_np,index_np))
    right_p=np.vstack((right_np,index_np))

    point=cv2.triangulatePoints(P1,P2,left_p,right_p)
    point=point/point[3]
    '''
    if np.size(right)<np.size(left):
        left=left[:,np.shape(left)[1]-np.shape(right)[1]:]
        point=cv2.triangulatePoints(P1,P2,left,right)
        point=point/point[3]'''

    return point