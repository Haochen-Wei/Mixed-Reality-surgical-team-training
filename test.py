import cv2
import psm_arm as P
from ambf_client import Client
from PyKDL import Frame, Rotation, Vector
import rospy
import time

#c = Client()

#c.connect()

#psm1=P.PSM(c,"psm1")
#psm2=P.PSM(c,"psm2")
cap = cv2.VideoCapture("/dev/video2")


key=''
while key != 113:
    key=cv2.waitKey(5)
    retval, frame = cap.read()
    print(retval)
    #psm1.servo_cp(Frame(Rotation.RPY(3.14, 0, 1.5707), Vector(0.0, 0.0, -1.0)))
    #psm2.servo_cp(Frame(Rotation.RPY(3.14, 0, 1.5707), Vector(0.0, 0.0, -1.0)))
    #time.sleep(0.5)
    cv2.imshow("AAA",frame)


# input("click to update")
# psm1.servo_cp(Frame(Rotation.RPY(0.0, 0.0, 0.0), Vector(3.14, 0, 1.5707)))
