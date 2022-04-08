#!/usr/bin/env python3

import rospy
import actionlib
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import time

ANG_MAX = math.pi/18
VEL_MAX = 0.15

eps = 0.0001
my_pose, left_pose, right_pose = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
my_vel, left_vel, right_vel = 0.0, 0.0, 0.0
my_omega = 0.0
k=1

start = time.time()
time_list = []
path_list = []

def velocity_convert(theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    gain_ang = 1 #modify if necessary
    
    ang = math.atan2(vel_y, vel_x)
    if ang < 0:
        ang += 2 * math.pi
    
    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin =  min(max(vel_x, -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang

def callback_odom(data):
    '''
    Get robot data
    '''
    my_pose[0] = data.pose.pose.position.x
    my_pose[1] = data.pose.pose.position.y
    quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    _, _, my_pose[2] = euler_from_quaternion(quaternion)
    my_vel = data.twist.twist.linear.x
    my_omega = data.twist.twist.angular.z
    print("*"*10,my_vel, "*"*10)

    time_list.append(time.time()-start)
    path_list.append((my_pose[0], my_pose[1]))

def callback_left_odom(data):
    '''
    Get left robot data
    '''
    left_pose[0] = data.pose.pose.position.x
    left_pose[1] = data.pose.pose.position.y
    quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    _, _, left_pose[2] = euler_from_quaternion(quaternion)
    left_vel = data.twist.twist.linear.x

def callback_right_odom(data):
    '''
    Get right robot data
    '''
    right_pose[0] = data.pose.pose.position.x
    right_pose[1] = data.pose.pose.position.y
    quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    _, _, right_pose[2] = euler_from_quaternion(quaternion)
    right_vel = data.twist.twist.linear.x

def plotData(time_list, path_list):
    wpX = [x[0] for x in path_list]
    wpY = [x[1] for x in path_list]

    plt.plot(time_list, wpX)
    plt.xlabel("Time")
    plt.ylabel("X-coordinate")
    plt.title("X vs T")
    plt.grid()
    plt.show()

    plt.plot(wpX, wpY)
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.title("Robot Path")
    plt.grid()
    plt.show()

rospy.init_node('balancing', anonymous = True, disable_signals = True)
rospy.Subscriber('/odom', Odometry, callback_odom) #topic name fixed
rospy.Subscriber('/left_odom', Odometry, callback_left_odom) #topic name fixed
rospy.Subscriber('/right_odom', Odometry, callback_right_odom) #topic name fixed

pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
r = rospy.Rate(30)
steps = 0

#convert velocity vector to linear and angular velocties using velocity_convert function given above
while not rospy.is_shutdown():
    vel = [0,0]
    rightdist = right_pose[0] - my_pose[0]
    leftdist = my_pose[0] - left_pose[0]
    # print(rightdist, leftdist)
    vel[0] = k*(rightdist - leftdist)
    v_lin, v_ang = velocity_convert(my_pose[2], vel[0],vel[1])

    vel_msg = Twist()
    vel_msg.linear.x = v_lin
    vel_msg.angular.z = 0
    pub_vel.publish(vel_msg)
    print(v_lin, vel[0])
    r.sleep()

    if (abs(my_vel)<eps and abs(left_vel)<eps and abs(right_vel)<eps) and steps>1000:
        rospy.signal_shutdown("Balanced")
        break
    
    steps+=1

#store robot path with time stamps (data available in odom topic)

plotData(time_list, path_list)