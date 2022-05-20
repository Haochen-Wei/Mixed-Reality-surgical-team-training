import cv2
import numpy as np
import filter
import crop
import recon_point as rp
import socket

vid=cv2.VideoCapture(0)
IP="127.0.0.1"
UDP_PORT = 5005
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

while True:
    ret,graph=vid.read()
    #cv2.imshow("stereo",graph)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
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
    MESSAGE=b"123"
    sock.sendto(MESSAGE, (IP, UDP_PORT))
vid.release()
cv2.destroyAllWindows()