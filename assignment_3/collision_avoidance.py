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
import numpy as np

VEL_MAX = 0.15
ANG_MAX = math.pi/18

rospy.init_node('collision_avoidance', anonymous = True, disable_signals = True)

params = {}

params['vel_d'] = 15
params['ang_d'] = 20
params['obs_radi'] = 2*0.15

params['origin'] = (0.0, 0.0)
params['goal'] = (5.0, 0.0)
params['eps'] = 0.5
params['eps_ang'] = 0.0001

params['angles_cone'] = np.zeros(3)
params['obs_pose_x'] = params['obs_pose_y'] = np.ones(3)
params['obs_pose_vx'] = params['obs_pose_vy'] = np.ones(3)

params['my_pose'] = [0.0, 0.0, 0.0]
params['my_vel'] = [0.0, 0.0]
params['my_omega'] = 0.0

params['samples'] = []
params['feas_pts'] = []

params['dev_min'], params['dev_min_pt'] = 2*math.pi, None
params['dev_max'], params['dev_max_pt'] = 0, None

params['rate'] = rospy.Rate(30)

params['pub_vel'] = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 10)
params['start'] = time.time()
params['time_list'] = []
params['path_list'] = []

def velocity_convert(theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    gain_ang = 1 #modify if necessary
    
    ang = math.atan2(vel_y, vel_x)
    if ang < 0:
        ang += 2 * math.pi
    
    ang_err = min(max(ang - theta, - ANG_MAX), ANG_MAX)

    v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), - VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang

def callback_odom(data):
    '''
    Get robot data
    '''

    params['my_pose'][0] = data.pose.pose.position.x
    params['my_pose'][1] = data.pose.pose.position.y

    params['my_vel'][0] = data.twist.twist.linear.x
    params['my_vel'][1] = data.twist.twist.linear.y
    params['my_omega'] = data.twist.twist.angular.z

    quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    _, _, params['my_pose'][2] = euler_from_quaternion(quaternion)

    params['time_list'].append(time.time() - params['start'])
    params['path_list'].append((params['my_pose'][0], params['my_pose'][1]))

def clear_obs_params():
	params['obs_pose_x'] = params['obs_pose_y'] = np.zeros(3)
	params['obs_pose_vx'] = params['obs_pose_vy'] = np.zeros(3)
	# return params

def update_obs_params(obstacle, i):
	params['obs_pose_x'][i], params['obs_pose_y'][i] = obstacle.pose_x, obstacle.pose_y
	params['obs_pose_vx'][i], params['obs_pose_vy'][i] = obstacle.vel_x, obstacle.vel_y

def callback_obs(data):
	i = 0
	for obstacle in data.obstacles:
		update_obs_params(obstacle, i)
		i+=1

def euclid_dist(x1, x2):
	return ((x1[0]-x2[0])**2 + (x1[1]-x2[1])**2)**0.5

def goal_termination():
	return euclid_dist(params['goal'], params['my_pose'][:2]) < params['eps']

def radial_to_cartesian(vel, ang):
	return (params['origin'][0]+vel*np.cos(ang), params['origin'][1]+vel*np.sin(ang))

def gen_samples():
	theta = params['my_pose'][2]
	params['samples'] = []
	for vel in np.linspace(0, VEL_MAX, params['vel_d']):
		for ang in np.linspace(theta-ANG_MAX, theta+ANG_MAX, params['ang_d']):
			if vel!=0:
				params['samples'].append(radial_to_cartesian(vel, ang))

def gen_coll_cone(i):
	origin = params['my_pose'][:2]
	centre_obs = (params['obs_pose_x'][i], params['obs_pose_y'][i])
	obs_dist = ((origin[0]-centre_obs[0])**2+(origin[1]-centre_obs[1])**2)**0.5
	print(i, centre_obs, obs_dist)
	if params['obs_radi'] > obs_dist:
		return math.pi/2
	cone_angle = np.arcsin(params['obs_radi']/obs_dist)
	return cone_angle

def all_coll_cones():
	for i in range(len(params['obs_pose_x'])):
		params['angles_cone'][i] = gen_coll_cone(i)

def ang_vectors(a, b):
	if not (np.linalg.norm(a) and np.linalg.norm(b)):
		return 0
	return np.arccos(np.dot(a, b)/(np.linalg.norm(a)*np.linalg.norm(b)))

def coll_with_cone(pt, i):
	v_rel = (pt[0]-params['obs_pose_vx'][i], pt[1]-params['obs_pose_vy'][i])
	obs_vector = (params['obs_pose_x'][i]-params['my_pose'][0], params['obs_pose_y'][i]-params['my_pose'][1])
	theta = ang_vectors(v_rel, obs_vector)
	return abs(theta) < abs(params['angles_cone'][i])

def open_area(pt):
	for i in range(len(params['obs_pose_x'])):
		if coll_with_cone(pt,i):
			return False
	return True

def filter_points():
	cones = all_coll_cones()
	params['feas_pts'] = []
	for pt in params['samples']:
		if open_area(pt):
			params['feas_pts'].append(pt)

	for pt in params['feas_pts']:
		print(pt)

def opti_point():
	goal_dir = (params['goal'][0] - params['my_pose'][0], params['goal'][1]-params['my_pose'][1])
	params['dev_min'], params['dev_min_pt'] = 2*math.pi, None
	
	for pt in params['feas_pts']:
		curr_angle = ang_vectors(goal_dir, pt)
		print(curr_angle)
		if curr_angle<params['dev_min']:
			params['dev_min'] = curr_angle
			params['dev_min_pt'] = pt
		print(params['dev_min_pt'])

def proceed(towards_goal=False):
	v_lin, v_ang = velocity_convert(params['my_pose'][2], params['dev_min_pt'][0], params['dev_min_pt'][1])
	vel_msg = Twist()
	vel_msg.linear.x = v_lin
	vel_msg.angular.z = v_ang
	params['pub_vel'].publish(vel_msg)
	params['rate'].sleep()

def avoid_obs():
	while not goal_termination():
		gen_samples()
		filter_points()
		opti_point()
		proceed()

def plotData(time_list, path_list):
    wpX = [x[0] for x in path_list]
    wpY = [x[1] for x in path_list]

    size = min(len(wpX), len(time_list))

    wpX, wpY, time_list = wpX[:size], wpY[:size], time_list[:size]

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

rospy.Subscriber('/obs_data', ObsData, callback_obs)
rospy.Subscriber('/bot_1/odom', Odometry, callback_odom)

avoid_obs()
plotData(params['time_list'], params['path_list'])
