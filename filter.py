#This function is used to find the middle line of the tools.
import numpy as np
import cv2
from numba import jit
import time

#This function get the all centerline point (raw data) from the graph
@jit(nopython=True)
def centerline(B):
    midpoint_list=[]
    for i in range(int(B.shape[0])):
        start=[]
        end=[]
        flag=0
        for j in range(B.shape[1]-1):
            if B[i][j-1]-B[i][j]>10:#if B[i][j]==0 and B[i][j-1]!=0:
                start.append(j)
                flag=1
            if flag==1:
                if B[i][j+1]-B[i][j]>10:#if B[i][j]==0 and B[i][j+1]!=0:
                    end.append(j)
                    flag=0
        mid=[]
        if len(start)==len(end):
            for n in range(len(start)):
                if abs(start[n]-end[n])>5:
                    mid.append(int((start[n]+end[n])/2))
        else: 
            for n in range(len(start)-1):
                if abs(start[n]-end[n])>5:
                    mid.append(int((start[n]+end[n])/2))
        midpoint_list.append(mid)
    return midpoint_list

#Calculate the line of the graph
def get_line(img,mid_list,fixed_list):

    #Put the obtained points into a new binary graph
    new_img=np.zeros((np.shape(img)[0],np.shape(img)[1]),np.uint8)
    for i in range(len(mid_list)):
        #if len(mid_list[i])<=round(avg_len):
        for j in range(len(mid_list[i])):
            new_img[i][mid_list[i][j]]=255
            new_img[i][mid_list[i][j]+1]=255
            new_img[i][mid_list[i][j]+2]=255
            #new_img[i][mid_list[i][j]+3]=255
            if i>3:
                new_img[i-1][mid_list[i][j]]=255 
                new_img[i-2][mid_list[i][j]]=255 
                #new_img[i-3][mid_list[i][j]]=255 

            new_img[i][mid_list[i][j]-1]=255
            new_img[i][mid_list[i][j]-2]=255
            #new_img[i][mid_list[i][j]-3]=255

    #Find the line using line detection:
    lines_point=cv2.HoughLinesP(new_img,1,np.pi/180,75,50,50)#200,100,100
    rep_lines=[]
    #Using matrix method to calculate the parameter of the line
    for i in range(2):
        if  lines_point is not None:
            break
        lines_point=cv2.HoughLinesP(new_img,1,np.pi/180,50,50,100)
    

    #Coordinate here y axis in the image was independent variable and x axis is dependent variable
    if lines_point is not None:
        for i in range(len(lines_point)):            
            l = lines_point[i][0]
            Y=np.array([[l[0]],[l[2]]])#dependent variable (This Y not represent to the y axis)
            X=np.array([[l[1],1],[l[3],1]])#independent variable (This X not represent to the x axis)
            res=np.linalg.solve(X,Y)
            k=res[0][0]
            b=res[1][0]
            rep_lines.append([k,b])


    #Remove the repeat lines and return it
    lines=[]
    lines_index=[]
    for i in range(len(rep_lines)):
        tested=rep_lines.pop()
        A=np.array(tested)
        flag=0
        for j in rep_lines:
            B=np.array(j)
            if np.linalg.norm(A-B)<50:
                flag=1
                break
        if flag==0:
            lines.append(tested)
            for n in range(len(fixed_list)):
                fixed_point=fixed_list[n]
                if abs(fixed_point[0]-tested[0]*fixed_point[1]-tested[1])<5:
                    lines_index.append(n)
                    break
    return lines,lines_index


#Only take the point that are close to the line
def classify_point(mid_list,lines):
    filtered_point=[]
    #Exam the point one by one and keep the point that are close to the line
    for k in range(len(lines)):
        selected_line = lines[k]

        point_for_this_line = []
        
        for i in range(len(mid_list)):
            middle_this_line=0
            for j in range(len(mid_list[i])):
                if abs(mid_list[i][j]-(selected_line[0]*i+selected_line[1]))<5:
                    middle_this_line+=mid_list[i][j]   
            if middle_this_line!=0:
                middle_this_line/len(mid_list[i])
                point_for_this_line.append([middle_this_line,i])
        filtered_point.append(point_for_this_line)

    return filtered_point
    

#Drawing the raw point for comparision, keep for comparision
def draw_line(img,mid_list):
    #Draw the point
    for i in range(len(mid_list)):
        for j in range(len(mid_list[i])):
            cv2.circle(img,(mid_list[i][j],i),1,(0,0,255),0)


'''
#Test code below
#Import the graph start timer
img=cv2.imread('C:/Users/robin/Documents/Graph/Test3.jpg')
start=time.time()

#Get binary
gray=cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
[_,Binary]=cv2.threshold(gray,100,255,cv2.THRESH_BINARY)

#Get line data and draw the point
centerline_raw=centerline(Binary)
lines=get_line(img,centerline_raw)
line_dot=draw_dots(centerline_raw,lines)

#Show the time
end=time.time()
print('Time consuming is',end-start)

print(len(line_dot))
#Show the graph
cv2.imshow('a',img)
cv2.waitKey()'''