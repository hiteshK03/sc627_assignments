#!/usr/bin/env python3

from sc627_helper.msg import MoveXYGoal, MoveXYResult, MoveXYAction
import rospy
import actionlib
import matplotlib.pyplot as plt
import math
import time
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

def check_dist(curr_pos, obstacle, params):
    return curr_pos, params
    # SOME EXTRA CODE TO BRING BOT CLOSE TO OBSTACLE
    # if computeDistancePointToPolygon(obstacle, curr_pos)[0] > step_size:
    #     print("Adjusting")
    #     a, b = computeTangentVectorToPolygon(obstacle, curr_pos)
    #     last_pos = curr_pos
    #     curr_pos = curr_pos.incr_slope(-b,a,step_size)
    #     curr_pos = update_pos(last_pos, curr_pos, client)
    #     params = update_params(params, curr_pos)
    # return curr_pos, params

def bug_base_algo(start, goal, obstacles_list, step_size):
    rospy.init_node('test', anonymous=True, disable_signals=True)
    client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
    client.wait_for_server()

    params = get_params(goal, step_size, client)
    curr_pos = start
    params = update_params(params, curr_pos)

    while curr_pos.dist(goal) > step_size:
        d_min = float("inf")
        for i in obstacles_list:
            d, _, _ = computeDistancePointToPolygon(i, curr_pos)
            if d < d_min:
                d_min = d

        if d_min < step_size:
            print("Failure: There is an obstacle lying between the start and the goal\n")
            rospy.signal_shutdown("Reached the goal")
            return params

        last_pos = curr_pos
        curr_pos = curr_pos.incr_point(goal, step_size)
        curr_pos = update_pos(last_pos, curr_pos, client)
        params = update_params(params, curr_pos)

    params = update_params(params, goal)
    print("Success")
    rospy.signal_shutdown("Reached the goal")
    return params

def map_polygon(p_hit, obstacle, params):
    print("Started circum-navigation")
    curr_pos = p_hit
    goal = params['goal']
    step_size = params['step_size']
    client = params['client']
    d_min = p_hit.dist(goal) 
    p_leave = curr_pos
    count = 0
    w_prev = 0

    while curr_pos.dist(p_hit) > 2*step_size or count < 3:
        print("In circum-navigation")
        count+=1
        d = curr_pos.dist(goal)
        if d < d_min:
            d_min = d
            p_leave = curr_pos
        a, b = computeTangentVectorToPolygon(obstacle, curr_pos)
        _, w, _ = computeDistancePointToPolygon(obstacle, curr_pos)
        if w_prev != 0 and w==0:
            curr_pos, params = check_dist(curr_pos, obstacle, params)
        w_prev = w
        last_pos = curr_pos
        curr_pos = curr_pos.incr_slope(a,b,step_size)
        curr_pos = update_pos(last_pos, curr_pos, client)
        params = update_params(params, curr_pos)
    print("Ending circum-navigation")
    return p_leave, curr_pos, params

def check_failure(curr_pos, obstacle, params):
    centroid = poly_centroid(obstacle)
    v1 = Point(curr_pos.x-centroid.x, curr_pos.y-centroid.y)
    v2 = Point(curr_pos.x-params['goal'].x, curr_pos.y-params['goal'].y)
    dot_p = v1.x*v2.x + v1.y*v2.y
    return dot_p

def travel_leave_point(p_leave, curr_pos, obstacle, params):
    print("Going to closest point")
    goal = params['goal']
    step_size = params['step_size']
    client = params['client']
    w_prev = 0
    while computeDistancePointToSegment(curr_pos, p_leave, goal)[0] > step_size:
        print("Going to closest point")
        a, b = computeTangentVectorToPolygon(obstacle, curr_pos)
        _, w, _ = computeDistancePointToPolygon(obstacle, curr_pos)
        if w_prev != 0 and w==0:
            curr_pos, params = check_dist(curr_pos, obstacle, params)
        w_prev = w
        last_pos = curr_pos
        curr_pos = curr_pos.incr_slope(a,b,step_size)
        curr_pos = update_pos(last_pos, curr_pos, client)
        params = update_params(params, curr_pos)
    dot_p = check_failure(curr_pos, obstacle, params)
    if dot_p > 0:
        rospy.signal_shutdown("Failure")
    return curr_pos, params

def bug_1_algo(start, goal, obstacles_list, step_size):
    rospy.init_node('test', anonymous=True)
    client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
    client.wait_for_server()
    params = get_params(goal, step_size, client)
    curr_pos = start
    params = update_params(params, curr_pos)
    while curr_pos.dist(goal) > step_size:
        d_min = float("inf")
        for i in obstacles_list:
            d, _, _ = computeDistancePointToPolygon(i, curr_pos)
            if d < d_min:
                d_min = d
                i_min = i
        
        if d_min < step_size:
            p_leave, curr_pos, params = map_polygon(curr_pos, i_min, params)
            curr_pos, params = travel_leave_point(p_leave, curr_pos, i_min, params)
        
        last_pos = curr_pos
        curr_pos = curr_pos.incr_point(goal, step_size)
        curr_pos = update_pos(last_pos, curr_pos, client)
        params = update_params(params, curr_pos)
    params = update_params(params, goal)
    print(params['time'][-1])
    print("Success")
    return params

def plot_graphs(params, obs_list, algo):
    X, Y = [], []
    for i in params['path']:
        X.append(i.x)
        Y.append(i.y)

    plt.scatter(X, Y, c="blue",s=4)
    t1 = plt.Polygon(obs_list[0], color="red", fill=False)
    plt.gca().add_patch(t1)

    t2 = plt.Polygon(obs_list[1], color="red", fill=False)
    plt.gca().add_patch(t2)
    plt.xlabel('Time (s)', fontsize=10)
    plt.ylabel('Distance from Goal (m)', fontsize=10)
    plt.savefig("src/sc627_assignments/assignment_1/path-bug_"+algo+".png")
    plt.show()
    plt.plot(params['time'], params['dist'], c="blue")
    plt.savefig("src/sc627_assignments/assignment_1/time-dist-bug_"+algo+".png")
    plt.show()

def write_to_file(params, algo):
    filename = "src/sc627_assignments/assignment_1/output_"+algo+".txt"
    file = open(filename, "w")
    for i in params['path']:
        file.write(str(i.x)+", "+str(i.y)+"\n")
    file.close()