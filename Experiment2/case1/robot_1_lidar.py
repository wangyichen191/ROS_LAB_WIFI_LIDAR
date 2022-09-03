#!/usr/bin/env python
import rospy
import numpy
import math
import sys
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Accel

#global var
start_distance = 0.5
safe_distance = 0.5
cos = 0
has_ros = 0
car_width = 0.2


def deal_angle(angle):
        if angle < math.pi:
                return angle
        else:
                return angle - 2 * math.pi


def judge_object(distance,LastDistance):
        #judge whether two point belong to the same object
        global cos
        result = []
        dis = math.sqrt(distance**2 + LastDistance**2 - 2*distance*LastDistance*cos)
        if dis > 0.03:
                result = [0,0]
        else:
                result = [dis,1]
        return result

def Call_Check(self,arrays, scan_data):
        #judge the leader's position according to the point cloud
        result = []
        filter_result = []
        for array in arrays:
                if array[3] < car_width + 0.2 :
                        #and array[0] < safe_distance + 1 and array[0] > safe_distance - 0.4:
                        angle = (array[1] + array[2]) * scan_data.angle_increment / 2
                        #follow the car which lie in the left front or right front of the follower 
                        if True:
                                array.append(angle)
                                filter_result.append(array)

        for array in filter_result:
                temp_dis1 = array[0] - self.lastdistance1
                temp_ang1 = deal_angle(array[4]) - self.lastangle1
                temp_dis2 = array[0] - self.lastdistance2
                temp_ang2 = array[4] - self.lastangle2

                array.append(numpy.abs(temp_ang1))
                array.append(numpy.abs(temp_dis1))
                array.append(numpy.abs(temp_ang2))
                array.append(numpy.abs(temp_dis2))
                array.append(array[5]*0.5+array[6]*0.5)
                array.append(array[7]*0.5+array[8]*0.5)
        filter_result.sort(key = lambda x:x[9],reverse=False)
        if len(filter_result) == 0:
                result.append([20,'No Filter Result'])
        else:
                if filter_result[0][5] > 0.2:
                        result.append([20,'The Angle Out of Prediction',filter_result])
                elif filter_result[0][6] > 0.5:
                        result.append([20,'The Distance Out of Prediction'])
                else:
                        self.lastdistance1 = filter_result[0][0]
                        self.lastangle1 = deal_angle(filter_result[0][4])
                        result.append(filter_result[0])
        filter_result.sort(key = lambda x:x[10],reverse=False)
        if len(filter_result) == 0:
                #return None
                result.append([20,'No Filter Result'])
        else:
                if filter_result[0][7] > 0.2:
                        result.append([20,'The Angle Out of Prediction',filter_result])
                elif filter_result[0][8] > 0.5:
                        result.append([20,'The Distance Out of Prediction'])
                else:
                        self.lastdistance2 = filter_result[0][0]
                        self.lastangle2 = filter_result[0][4]
                        result.append(filter_result[0])
        return result

class laserTracker:
        def __init__(self):
                self.acc = 0
                self.leaderlinear = 0
                self.leaderangle = 0
                self.mylinear = 0
                self.myangle = 0
                self.followerlinear = 0
                self.followerangle = 0
                self.lastangle1 = 0
                self.lastdistance1 = safe_distance
                self.lastangle2 = math.pi
                self.lastdistance2 = safe_distance
                self.delta10 = 0
                self.delta12 = 0

                self.f = open('./../output/output_1.txt','w')
                self.f.write("test1 robot1\n")
                self.f.write("time(s) position_x(m) position_y(m) linear_speed(m/s) angular_speed(rad/s) acceleration(m/s^2)\n")

                self.scanSubscriber = rospy.Subscriber('/robot_1/scan',LaserScan,self.ScanCallback)
                self.Publisher = rospy.Publisher('/robot_1/cmd_vel',Twist,queue_size=1)
                self.mySubscriber = rospy.Subscriber('/robot_1/odom',Odometry,self.MyCallback)

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
                                        distance_result = judge_object(distance,LastDistance)
                                        if distance_result[1] == 1:
                                                dis_seq[k][2] = i
                                                dis_seq[k][3] = dis_seq[k][3] + distance_result[0]
                                                dis_seq[k][0] = (dis_seq[k][0] + distance) / 2
                                        else:
                                                temp = [distance,i,i,0]
                                                dis_seq.append(temp)
                                                k = k + 1
                                        LastDistance = distance
                result = Call_Check(self,dis_seq,scan_data)

                if(result[0][0]!=20):
                        angle = ((result[0][1] + result[0][2]) * scan_data.angle_increment) / 2
                        angle_result = deal_angle(angle)
                        distance = result[0][0]
                        msg = Twist()
                        msg.linear.x =  (distance - safe_distance) * 1.3
                        msg.angular.z = 0.7 * angle_result
                        self.Publisher.publish(msg)
                        rospy.loginfo("Distance_leader: %0.2f Angle_Leader: %0.2f rad Linear Speed: %0.2f m/s Angle Speed: %0.2f rad/s Accel: %0.2f",result[0][0],angle_result,msg.linear.x,msg.angular.z,self.acc)
                else:
                        msg = Twist()
                        msg.linear.x = self.mylinear
                        msg.angular.z = self.leaderangle
                        self.Publisher.publish(msg)
                        rospy.loginfo("Linear Speed: %0.2f m/s Angle Speed: %0.2f rad/s",msg.linear.x,msg.angular.z)

        def MyCallback(self,data):
                self.mylinear = data.twist.twist.linear.x
                self.myangle = data.twist.twist.angular.z

                ticks = time.time()
                position_x = data.pose.pose.position.x - start_distance
                position_y = data.pose.pose.position.y
                linear_speed = data.twist.twist.linear.x
                angular_speed = data.twist.twist.angular.z
                accel = 0
                self.f.write(str(ticks) + " " + str(position_x) + " " + str(position_y) + " " + str(linear_speed) + " " + \
                        str(angular_speed) + " " + str(accel) + "\n") 

def main():
        rospy.init_node('robot_1_lidar',anonymous=True)
        laserTracker()
        rospy.spin()

if __name__ == "__main__":

        main()
