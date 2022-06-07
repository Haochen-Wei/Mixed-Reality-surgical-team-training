import numpy as np
import cv2

#Read the difference image and white balance
background=cv2.imread("background.jpg")
img_tool=cv2.imread("img_tool.jpg")
g_back=cv2.cvtColor(background,cv2.COLOR_RGB2GRAY)
g_tool=cv2.cvtColor(img_tool,cv2.COLOR_RGB2GRAY)
max_back=np.amax(g_back)
max_tool=np.amax(g_tool)
g_back=np.int16(g_back*(255/max_back))
g_tool=np.int16(g_tool*(255/max_tool))

#Find the 
diff=np.uint8(np.abs(g_back-g_tool))

cv2.imshow('diff',diff)
cv2.waitKey()
