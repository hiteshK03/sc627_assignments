#!/usr/bin/env python3
import sys
sys.path.append("..")
import time
import rospy
import actionlib
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
from helper import *

def read_file(filename):
	file = open(filename, "r")
	data = file.readlines()
	
	line = data[0].rstrip()
	start_x, start_y = [float(i) for i in line.split(',')]
	start = Point(start_x, start_y)
	
	line = data[1].rstrip()
	goal_x, goal_y = [float(i) for i in line.split(',')]
	goal = Point(goal_x, goal_y)
	
	line = data[2].rstrip()
	step_size = float(line)
	
	obs_data = data[3:]
	obs_list = []
	temp = []
	for i, obs in enumerate(obs_data):
		if obs=="\n":
			temp = []
			if i!=1:
				obs_list.append(temp)
			continue
		obs = obs.split('\n')
		for j in obs:
			if not j=="":
				edge_x, edge_y = [float(x) for x in j.split(',')]
				temp.append((edge_x, edge_y))

	return start, goal, obs_list, step_size

def get_params(goal, step_size, client):
    params = {}
    params['start_time'] = time.time()
    params['goal'] = goal
    params['step_size'] = step_size
    params['client'] = client
    params['dstar'] = 1.5
    params['chi'] = 0.8
    params['qstar'] = 2
    params['eta'] = 0.8
    params['grad'] = Point(1,1)
    params['total_dist'] = 0
    params['path'] = []
    params['time'] = []
    params['dist'] = []
    return params

def update_params(params, curr_pos):
    params['path'].append(curr_pos)
    params['time'].append(time.time()-params['start_time'])
    params['dist'].append(curr_pos.dist(params['goal']))
    params['total_dist'] += params['step_size']
    return params

def update_pos(last_pos, new_pos, client):
    print("Goal : ", new_pos.x, new_pos.y)
    go_to = MoveXYGoal()
    go_to.pose_dest.x = new_pos.x
    go_to.pose_dest.y = new_pos.y
    go_to.pose_dest.theta = math.atan2((new_pos.y-last_pos.y),(new_pos.x-last_pos.x))
    client.send_goal(go_to)
    client.wait_for_result()
    result = client.get_result()
    print("Result : ", result.pose_final.x, result.pose_final.y)
    return Point(result.pose_final.x, result.pose_final.y)

def compute_gradient(obstacles_list, params):
    goal = params['goal']
    curr_pos = params['path'][-1]
    dstar = params['dstar']
    chi = params['chi']
    qstar = params['qstar']
    eta = params['eta']
    if goal.dist(curr_pos) <= dstar:
        factor = chi
    else:
        factor = dstar*chi/goal.dist(curr_pos)
    attr_grad = (goal-curr_pos)*factor
    rep_grad = Point(0,0)

    for obs in obstacles_list:
        d, _, _ = computeDistancePointToPolygon(obs, curr_pos)
        if d < qstar:
            normal = computeNormalVectorToPolygon(obs, curr_pos)
            rep_grad = rep_grad + Point(-1*normal[0], -1*normal[1])*eta*(-1/qstar + 1/d) * (1/d**2)

    grad = attr_grad + rep_grad
    grad = grad / grad.dist(None) 
    params['grad'] = grad
    return params

def move(obstacles_list, params, brake=False):
    if not brake:
        params = compute_gradient(obstacles_list, params)
    last_pos = params['path'][-1]
    grad = params['grad']
    curr_pos = last_pos.incr_slope(grad.x, grad.y, params['step_size'])
    curr_pos = update_pos(last_pos, curr_pos, params['client'])
    params = update_params(params, curr_pos)
    return params

def break_loop(obstacles_list, params):
    for i in range(3):
        print("Towards goal")
        grad = params['goal']-params['path'][-1]
        grad /= grad.dist(None)
        params['grad'] = grad
        params = move(obstacles_list, params, True)
    return params

def potential_planner(start, goal, obstacles_list, step_size):
    rospy.init_node('test', anonymous=True)
    client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
    client.wait_for_server()

    params = get_params(goal, step_size, client)
    curr_pos = start
    params = update_params(params, curr_pos)
    i = 0
    while not params['path'][-1].dist(goal) < 0.1:
        params = move(obstacles_list, params)
        if len(params['path'])>3:
            for i in range(3):
                if params['path'][-1].dist(params['path'][-2-i]) < 0.04:
                    print("Break")
                    params = break_loop(obstacles_list, params)
    print("Success")
    return params

def plot_graphs(params, obs_list):
    X, Y = [], []
    for i in params['path']:
        X.append(i.x)
        Y.append(i.y)

    plt.scatter(X, Y, c="blue",s=4)
    t1 = plt.Polygon(obs_list[0], color="red", fill=False)
    plt.gca().add_patch(t1)

    t2 = plt.Polygon(obs_list[1], color="red", fill=False)
    plt.gca().add_patch(t2)
    plt.savefig("src/sc627_assignments/assignment_2/path.png")
    plt.show()

def write_to_file(params):
    filename = "src/sc627_assignments/assignment_2/output.txt"
    file = open(filename, "w")
    for i in params['path']:
        file.write(str(i.x)+", "+str(i.y)+"\n")
    file.close()

if __name__== '__main__':
    filename = "src/sc627_assignments/assignment_2/input.txt"
    start, goal, obs_list, step_size = read_file(filename)
    params = potential_planner(start, goal, obs_list, step_size)
    write_to_file(params)
    plot_graphs(params, obs_list)
    print("Total time taken: ", params['time'][-1])
    print("Total length covered: ", params['total_dist'])   
