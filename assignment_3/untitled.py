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

vel_d = 15
ang_d = 20

obs_radi = 2*0.15

def init_params(goal):
	my_pose = [0.0, 0.0, 0.0]
	my_vel = [0.0, 0.0]
	my_omega = 0.0