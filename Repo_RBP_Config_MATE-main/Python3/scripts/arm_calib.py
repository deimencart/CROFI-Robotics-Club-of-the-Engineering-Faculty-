#!/usr/bin/env python3
#coding=utf-8
import time
from Arm_Lib import Arm_Device
 
# Create a robotic arm object
Arm = Arm_Device()
time.sleep(.1)


# Middle servo
Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 180, 1000)
time.sleep(2)
# Turn off the torque, we can manually change the angle of all servos of the robotic arm 
Arm.Arm_serial_set_torque(0)


# After adjusting the angle of a certain servo, you can set the median deviation of a certain servo separately
id = 6
Arm.Arm_serial_servo_write_offset_switch(id)
time.sleep(.1)
state = Arm.Arm_serial_servo_write_offset_state()
if state == 1:
    print("set offset ok!")
elif state == 2:
    print("error! set offset overrun !")
elif state == 0:
    print("error! set offset error !")


# Set the median deviation of all servos (No.1-6)
for i in range(6):
    id = i + 1
    Arm.Arm_serial_servo_write_offset_switch(id)
    time.sleep(.1)
    state = Arm.Arm_serial_servo_write_offset_state()
    if state == 1:
        print("id:%d set offset ok!" % id)
    elif state == 2:
        print("error!id:%d set offset overrun !" % id)
    elif state == 0:
        print("error!id:%d set offset error !" % id)


# After the adjustment is complete, turn on the torque
Arm.Arm_serial_set_torque(0)


# Clear the median deviation of all servo settings and restore the default state.
# If you need to clear the median deviation of all servos, please delete the # below and run this unit
#Arm.Arm_serial_servo_write_offset_switch(0)

del Arm  # Release the Arm object



