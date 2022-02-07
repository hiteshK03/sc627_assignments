#!/usr/bin/env python3

from root import *

algo = 'base'
filename = "src/sc627_assignments/assignment_1/input.txt"
start, goal, obs_list, step_size = read_file(filename)
params = bug_base_algo(start, goal, obs_list, step_size)
write_to_file(params, algo)
plot_graphs(params, obs_list, algo)
print("Total time taken: ", params['time'][-1])
print("Total length covered: ", params['total_dist'])