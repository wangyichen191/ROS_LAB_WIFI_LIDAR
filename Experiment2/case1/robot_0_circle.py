#!/usr/bin/env python

import rospy
import sys
import os
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Circle:
        def __init__(self,linear=0.2,angular=0,accel=0):
                self.sub = rospy.Subscriber('/robot_0/odom',Odometry,self.callback)
                self.pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=1)
                self.f = open('./../output/output_1.txt','w')
                self.f.write("test1 robot0\n")
                self.f.write("time(s) position_x(m) position_y(m) linear_speed(m/s) angular_speed(rad/s) acceleration(m/s^2)\n")
                self.linear = linear
                self.angular = angular
                self.accel = accel
                self.publisher()

        def publisher(self):
                msg = Twist()
                r = rospy.Rate(50)
                while True:
                        msg.linear.x = self.linear
                        msg.angular.z = self.angular
                        self.pub.publish(msg)
                        r.sleep()
                    

        def callback(self,data):
                position_x = data.pose.pose.position.x
                position_y = data.pose.pose.position.y
                linear_speed = data.twist.twist.linear.x
                angular_speed = data.twist.twist.angular.z
                accel = self.accel
                self.f.write(str(position_x) + " " + str(position_y) + " " + str(linear_speed) + " " + \
                        str(angular_speed) + " " + str(accel) + "\n")    
            

def main():
        rospy.init_node('robot_0_circle',anonymous=True)
        len_argv = len(sys.argv)
        if len_argv == 1:
                linear = 0.2
                angular = 0
                accel = 0
        elif len_argv == 2:
                linear = sys.argv[1]
                angular = 0
                accel = 0
        elif len_argv == 3:
                linear = sys.argv[1]
                angular = sys.argv[2]
                accel = 0
        else:
                linear = sys.argv[1]
                angular = sys.argv[2]
                accel = sys.argv[3]
        Circle(linear,angular,accel)

if __name__ == "__main__":
        main()