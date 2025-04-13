#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
   # 360 / 10 = 36
    distance = 6 #set maximun distance (m)
    regions = [
        min(min(msg.ranges[0:35]), distance),
        min(min(msg.ranges[36:71]), distance),
        min(min(msg.ranges[72:107]), distance),
        min(min(msg.ranges[108:143]), distance),
        min(min(msg.ranges[144:179]), distance),
   #     'back_rigth': min(min(msg.ranges[180:215]), distance),
    #    'back_frigth': min(min(msg.ranges[216:251]), distance),
     #   'back_front': min(min(msg.ranges[252:287]), distance),
      #  'back_fleft': min(min(msg.ranges[288:323]), distance),
       # 'back_left': min(min(msg.ranges[324:359]), distance),
    ]
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
