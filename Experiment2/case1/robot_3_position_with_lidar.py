#!/usr/bin/env python

from ctypes import pointer
from faulthandler import disable
from turtle import distance
import rospy
import math
import numpy
import sys
import os
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

import robot_1_lidar

#global var

start_distance = 0.5
safe_distance = 0.5
frequency = 0.02
frequency_lidar = 0.2

has_ros=0
cos = 0

def calculate_distance(pos1,pos2):
        return math.sqrt((pos1.position.x-pos2.position.x)**2 + (pos1.position.y-pos2.position.y)**2)

def update_distance_vector(self):
        self.distance_vector = [self.pos0.position.x-self.pos1.position.x,self.pos0.position.y-self.pos1.position.y]

def update_robot1_vector(self,data):
        a = 2 * numpy.arccos(data.pose.pose.orientation.w)
        if data.pose.pose.orientation.z >= 0:
                b = 1
        else:
                b = -1
        self.robot1_vector = [numpy.cos(a),b*numpy.sin(a)]

def left_vertical(vector):
        z = numpy.array([0,0,1])
        x = numpy.array([vector[0],vector[1],0])
        y = numpy.cross(z,x)
        return [y[0],y[1]]

def calculate_angular_dis(self):
        cos = numpy.dot(self.distance_vector,self.robot1_vector) / (numpy.linalg.norm(self.distance_vector) * numpy.linalg.norm(self.robot1_vector))
        vector_vertical  = left_vertical(self.robot1_vector)
        symbol = numpy.dot(vector_vertical,self.distance_vector)
        if symbol >= 0:
                return numpy.arccos(cos)
        else:
                return numpy.arccos(cos)*(-1)

def check_distance(distance,data):
        global safe_distance
        global frequency
        dis = distance - safe_distance
        if dis > 0:
                return dis * 1.2 + data.twist.twist.linear.x
        else:
                return dis * 0.8 +  data.twist.twist.linear.x

def update_leader_follower(self,result,scan_data):
        if result[0][0] != 20:
                angle = ((result[0][1] + result[0][2]) * scan_data.angle_increment) / 2
                self.angle = robot_1_lidar.deal_angle(angle)
                distance = result[0][0]
                newpos0_x = self.pos1.position.x + distance * numpy.cos(angle)
                newpos0_y = self.pos1.position.y + distance * numpy.sin(angle)
                abl_distance = math.sqrt((self.lidar_pos0.position.x - newpos0_x)**2 + (self.lidar_pos0.position.y - newpos0_y)**2)
                self.lidar_twist0.linear.x = abl_distance / frequency_lidar
                self.lidar_pos0.position.x = newpos0_x
                self.lidar_pos0.position.y = newpos0_y

        if result[1][0] != 20:
                angle = ((result[1][1] + result[1][2]) * scan_data.angle_increment) / 2
                distance = result[1][0]
                newpos2_x = self.pos1.position.x + distance * numpy.cos(angle)
                newpos2_y = self.pos1.position.y + distance * numpy.sin(angle)
                abl_distance = math.sqrt((self.lidar_pos2.position.x - newpos2_x)**2 + (self.lidar_pos2.position.y - newpos2_y)**2)
                self.lidar_twist2.linear.x = abl_distance / frequency_lidar
                self.lidar_pos2.position.x = newpos2_x
                self.lidar_pos2.position.y = newpos2_y

class SubThenPub:
        def __init__(self,p,v):
                global start_distance
                self.distance_vector = numpy.array([1,0])
                self.robot1_vector = numpy.array([1,0])
                self.pos0 = Pose()
                self.pos1 = Pose()
                self.pos2 = Pose()
                self.twist0 = Twist()
                self.twist1 = Twist()
                self.twist2 = Twist()
                self.pos0.position.x = start_distance
                self.pos0.position.y = 0
                self.twist0.linear.x = 0
                self.twist0.angular.z = 0
                self.pos1.position.x = 0
                self.pos1.position.y = 0
                self.twist1.linear.x = 0
                self.twist1.angular.z = 0
                self.pos2.position.x = -start_distance
                self.pos2.position.y = 0
                self.twist2.linear.x = 0
                self.twist2.angular.z = 0

                self.lidar_pos0 = Pose()
                self.lidar_twist0 = Twist()
                self.lidar_pos0.position.x = start_distance
                self.lidar_pos0.position.y = 0
                self.lidar_twist0.linear.x = 0
                self.lidar_twist0.angular.z = 0

                self.lidar_pos2 = Pose()
                self.lidar_twist2 = Twist()
                self.lidar_pos2.position.x = -start_distance
                self.lidar_pos2.position.y = 0
                self.lidar_twist2.linear.x = 0
                self.lidar_twist2.angular.z = 0

                self.pos_leader = Pose()
                self.twist_leader = Twist()
                self.pos_leader.position.x = 2 * start_distance
                self.pos_leader.position.y = 0
                self.twist_leader.linear.x = 0
                self.twist_leader.angular.z = 0

                self.angle = 0

                self.accel = 0
                self.p = p
                self.v = v

                self.f = open('./../output/output_1.txt','w')
                self.f.write("test1 robot3\n")
                self.f.write("k_p:" + str(p) + " " + "k_v:" + str(v) + "\n")
                self.f.write("time(s) position_x(m) position_y(m) linear_speed(m/s) angular_speed(rad/s) acceleration(m/s^2)\n")
                self.pub = rospy.Publisher('/robot_3/cmd_vel',Twist,queue_size=1)

                # self.__sub_ = rospy.Subscriber('/robot_2/scan',Odometry,self.update_robot_leader,queue_size=1)
                self.sub0 = rospy.Subscriber('/robot_0/odom',Odometry,self.update_robot_leader_0,queue_size=1)
                self.followsub = rospy.Subscriber('/robot_2/odom',Odometry,self.update_robot_leader_2,queue_size=1)
                self.sub = rospy.Subscriber('/robot_3/scan',LaserScan,self.ScanCallback)
                self.selfsub = rospy.Subscriber('/robot_3/odom',Odometry,self.update_robot_me,queue_size=1)

        def ScanCallback(self,scan_data):
        # if lidar has found a car, output 'yes', else 'no'
                global has_ros
                dis_seq = []
                LastDistance = float('inf')
                k = -1

                if has_ros == 0:
                        global cos
                        cos = numpy.cos(scan_data.angle_increment)
                        has_ros = 1

                for i, distance in enumerate(scan_data.ranges):
                        if distance != float('inf'):
                                #we create the first dis_seq 
                                if LastDistance == float('inf'):
                                        temp = [distance,i,i,0]
                                        dis_seq.append(temp)
                                        k = k + 1
                                        LastDistance = distance
                                else:
                                        distance_result = robot_1_lidar.judge_object(distance,LastDistance)
                                        if distance_result[1] == 1:
                                                dis_seq[k][2] = i
                                                dis_seq[k][3] = dis_seq[k][3] + distance_result[0]
                                                dis_seq[k][0] = (dis_seq[k][0] + distance) / 2
                                        else:
                                                temp = [distance,i,i,0]
                                                dis_seq.append(temp)
                                                k = k + 1
                                        LastDistance = distance

                result = robot_1_lidar.Call_Check(self,dis_seq,scan_data)

                update_leader_follower(self,result,scan_data)

                pos0_pos3 = calculate_distance(self.pos_leader,self.pos1)
                pos2_pos3 = calculate_distance(self.pos1,self.pos0)

                self.accel = self.p * ((pos0_pos3 - 3 * start_distance) + (pos2_pos3 - start_distance)) + \
                             self.v * ((self.twist_leader.linear.x - self.twist1.linear.x) + (self.twist0.linear.x - self.twist1.linear.x))
                
                msg = Twist()
                msg.linear.x = self.twist1.linear.x + self.accel * frequency_lidar
                msg.angular.z = self.angle * 0.5
                self.pub.publish(msg)

                rospy.loginfo("pos0_pos3: %0.2f pos2_pos3: %0.2f x: %0.2f y:%0.2f linear_speed: %0.2f angular_speed: %0.2f", \
                              pos0_pos3,pos2_pos3,self.pos1.position.x,self.pos1.position.y,self.twist1.linear.x,self.twist1.angular.z)

                rospy.loginfo("LEADER: x:%0.2f y:%0.2f linear_speed:%0.2f angular_speed:%0.2f",self.pos0.position.x,self.pos0.position.y,self.twist0.linear.x,self.twist0.angular.z)
                #rospy.loginfo("FOLLOWER: x:%0.2f y:%0.2f linear_speed:%0.2f angular_speed:%0.2f",self.pos2.position.x,self.pos2.position.y,self.twist2.linear.x,self.twist2.angular.z)
                

        def update_robot_leader_0(self,data):
                self.pos_leader.position.x = data.pose.pose.position.x + 3 * start_distance
                self.pos_leader.position.y = data.pose.pose.position.y
                self.twist_leader.linear.x = data.twist.twist.linear.x
                self.twist_leader.angular.z = data.twist.twist.angular.z
                
        def update_robot_me(self,data):
                update_robot1_vector(self,data)
                self.pos1.position.x = data.pose.pose.position.x
                self.pos1.position.y = data.pose.pose.position.y
                self.twist1.linear.x = data.twist.twist.linear.x
                self.twist1.angular.z = data.twist.twist.angular.z

                ticks = time.time()
                position_x = data.pose.pose.position.x - 3*start_distance
                position_y = data.pose.pose.position.y
                linear_speed = data.twist.twist.linear.x
                angular_speed = data.twist.twist.angular.z
                accel = self.accel
                self.f.write(str(ticks) + " " + str(position_x) + " " + str(position_y) + " " + str(linear_speed) + " " + \
                        str(angular_speed) + " " + str(accel) + "\n") 

        def update_robot_leader_2(self,data):
                self.pos0.position.x = data.pose.pose.position.x + start_distance
                self.pos0.position.y = data.pose.pose.position.y
                self.twist0.linear.x = data.twist.twist.linear.x
                self.twist0.angular.z = data.twist.twist.angular.z

def main():
        rospy.init_node('robot_3_position_with_lidar',anonymous=True)
        len_argv = len(sys.argv)
        if len_argv == 1:
                SubThenPub()
        else:
                SubThenPub(p=float(sys.argv[1]),v=float(sys.argv[2]))
        rospy.spin()

if __name__ == "__main__":
        main()
        