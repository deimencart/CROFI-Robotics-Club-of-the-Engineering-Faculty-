#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from math import atan2
from tf.transformations import euler_from_quaternion

global xm
global ym
global thetam  



class NaviPoints():
    def __init__(self):
        self._position = Point()
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._go_to_point_callback)
        self._cmd_msg_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)


    def _go_to_point_callback(self, msg):
        obj = Point ()
        obj.x = 10
        obj.y = 10

        vel = Twist()   
        xm = msg.pose.pose.position.x
        ym = msg.pose.pose.position.y
        self.rot_q = msg.pose.pose.orientation
        ( _, _, thetam) = euler_from_quaternion([self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w])
        while (obj.x != xm) and (obj.x != ym): 
            vec_x = obj.x - xm
            vec_y = obj.y - ym
            angulo = atan2(vec_y,vec_x)
            rospy.loginfo(angulo)
            
            if abs(angulo - thetam) > 0.1: 
                rospy.loginfo(thetam)
                vel.linear.x = 0.0
                vel.angular.z = 0.3

            else: 
                vel.linear.x = 0.5
                vel.angular.z = 0.0
            
            self._cmd_msg_pub.publish(vel)
            r = rospy.Rate(10)
            r.sleep()
    
    def loop (self): 
        rospy.spin() 
            


def main():
    try:
        rospy.init_node('motion_planing')
        p2p = NaviPoints()
        p2p.loop()
        
    except rospy.ROSInterruptException as e:
        print(str(e))



if __name__ == '__main__':
    main()
