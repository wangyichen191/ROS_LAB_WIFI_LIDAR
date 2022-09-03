#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def main():
        rospy.init_node('circle_final',anonymous=True)
        pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=1)
        msg = Twist()
        r = rospy.Rate(50)
        while True:
                msg.linear.x = 0.2
                #msg.angular.z = 0.3
                pub.publish(msg)
                r.sleep()

if __name__ == "__main__":
        main()