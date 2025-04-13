#!/usr/bin/env python3
#coding=utf-8
from cv2 import *
import time
from Arm_Lib import Arm_Device
# Create a robotic arm object
Arm = Arm_Device()
time.sleep(.1)
Arm.Arm_serial_servo_write6(90  ,90 , 90, 90, 90, 0, 1000)
time.sleep(1)
namedWindow("webcam")
vc = VideoCapture(0);

while True:
    Arm.Arm_serial_servo_write6(180 ,90 , 90, 0, 90, 0, 1000)
    next, frame = vc.read()
    imshow("webcam", frame)
    if waitKey(50) >= 0:
        break;
