#!/usr/bin/env python3
#coding=utf-8
import time
from Arm_Lib import Arm_Device
# Create a robotic arm object
Arm = Arm_Device()
time.sleep(.1)
# Middle servo
Arm.Arm_serial_servo_write6(180 ,90 , 90, 45, 180, 0, 1000)
time.sleep(1)
Arm.Arm_serial_servo_write6(180 ,90 , 90, 0, 0, 0, 1000)
time.sleep(1)
Arm.Arm_serial_servo_write6(180 ,90 , 90, 45, 180, 0, 1000)
time.sleep(1)
Arm.Arm_serial_servo_write6(180 ,90 , 90, 0, 0, 0, 1000)
time.sleep(1)
Arm.Arm_serial_servo_write6(180 ,90 , 90, 45, 90, 0, 1000)
time.sleep(1)
Arm.Arm_serial_servo_write6(180 ,90 , 90, 0, 90, 0, 1000)
time.sleep(1)
Arm.Arm_serial_servo_write6(180 ,90 , 90, 45, 90, 0, 1000)
time.sleep(1)
Arm.Arm_serial_servo_write6(90  ,90 , 90, 90, 90, 0, 1000)
time.sleep(1)


# Turn off the torque, we can manually change the angle of 
