#!/usr/bin/env python



from ctypes import pointer

import rospy

import math

import numpy

import sys

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Pose

from geometry_msgs.msg import Quaternion





#global var

start_distance = 0.5

safe_distance = 0.5

frequency = 0.02



def calculate_distance(pos1,pos2):

        return math.sqrt((pos1.position.x-pos2.position.x)**2 + (pos1.position.y-pos2.position.y)**2)

def update_position_0(self,data):

        global start_distance

        self.pos0.position.x = start_distance + data.pose.pose.position.x

        self.pos0.position.y = data.pose.pose.position.y

        self.twist0.linear.x = data.twist.twist.linear.x

        self.twist0.angular.z = data.twist.twist.angular.z



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


class SubThenPub:

        def __init__(self):

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

                self.__pub_ = rospy.Publisher('/robot_1/cmd_vel',Twist,queue_size=1)

                self.__sub_ = rospy.Subscriber('/robot_0/odom',Odometry,self.callback,queue_size=1)

                self.selfsub = rospy.Subscriber('/robot_1/odom',Odometry,self.update_robot1,queue_size=1)

                self.followsub = rospy.Subscriber('/odom',Odometry,self.update_robot2,queue_size=1)

        def callback(self,data):

                global frequency

                update_position_0(self,data)

                update_distance_vector(self)

                angular_dis = calculate_angular_dis(self)

                angular_speed = angular_dis * 0.8

                pos0_pos1 = calculate_distance(self.pos0,self.pos1)

                pos1_pos2 = calculate_distance(self.pos1,self.pos2)

                acc = float(sys.argv[1]) * ((pos0_pos1 - start_distance) + (-pos1_pos2 + start_distance)) + float(sys.argv[2]) * (self.twist0.linear.x - self.twist1.linear.x + self.twist2.linear.x - self.twist1.linear.x)

                msg = Twist()

                msg.linear.x = self.twist1.linear.x + acc / 50

                msg.angular.z = self.twist0.angular.z + 0.5 * angular_dis

                self.__pub_.publish(msg)
                #rospy.loginfo("twist0.angular:%0.2f angular_dis:%0.2f",self.twist0.angular.z,angular_dis)

                #rospy.loginfo("angular_dis: %0.2f distance_dis: %0.2f pub velocity [%0.2f m/s,%0.2f rad/s] leader speed:%0.2f x:%0.2f y:%0.2f leader_pos:%0.2f follower_pos:%0.2f",angular_dis,distance,linear_speed,angular_speed,data.twist.twist.linear.x,data.pose.pose.position.x,data.pose.pose.position.y)

                rospy.loginfo("pos0_pos1:%0.2f pos1_pos2:%0.2f x:%0.2f y:%0.2f linear_speed:%0.2f angular_speed:%0.2f",pos0_pos1,pos1_pos2,self.pos1.position.x,self.pos1.position.y,msg.linear.x,msg.angular.z)

                rospy.loginfo("LEADER: x:%0.2f y:%0.2f linear_speed:%0.2f angular_speed:%0.2f",self.pos0.position.x,self.pos0.position.y,self.twist0.linear.x,self.twist0.angular.z)
                rospy.loginfo("FOLLOWER: x:%0.2f y:%0.2f linear_speed:%0.2f angular_speed:%0.2f",self.pos2.position.x,self.pos2.position.y,self.twist2.linear.x,self.twist2.angular.z)


        def update_robot1(self,data):
                # rospy.loginfo("robot_1")

                update_robot1_vector(self,data)

                self.pos1.position.x = data.pose.pose.position.x

                self.pos1.position.y = data.pose.pose.position.y

                self.twist1.linear.x = data.twist.twist.linear.x

                self.twist1.angular.z = data.twist.twist.angular.z

        def update_robot2(self,data):

                rospy.loginfo("robot_2")

                self.pos2.position.x = data.pose.pose.position.x - start_distance

                self.pos2.position.y = data.pose.pose.position.y

                self.twist2.linear.x = data.twist.twist.linear.x

                self.twist2.angular.z = data.twist.twist.angular.z



def main():

        rospy.init_node('position2',anonymous=True)

        SubThenPub()

        rospy.spin()



if __name__ == "__main__":

        main()


