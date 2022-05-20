#This function was used to crop one image into 2 from the centerline. 

import cv2
def crop(img):
    center=img.shape[1]/2
    left=img[1:img.shape[0],1:int(center)]
    right=img[1:img.shape[0],int(center):img.shape[1]]
    A=[left,right]
    return A