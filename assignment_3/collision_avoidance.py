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

class CollisionAvoidance():

    def __init__(self):

        self.VEL_MAX = 0.15
        self.ANG_MAX = math.pi/18

        self.VEL_DISCRETE = 15
        self.ANG_DISCRETE = 20

        self.OBS_RADIUS = 2*0.15

        self.myPose = [0.0, 0.0, 0.0]
        self.myVel = [0.0,  0.0]
        self.myOmega = 0.0

        self.obsPoseX = np.ones(3)
        self.obsPoseY = np.ones(3)
        self.obsPoseVx = np.ones(3)
        self.obsPoseVy = np.ones(3)

        self.coneAngles = np.zeros(3)

        self.samples = []
        self.feasiblePts = []

        self.goalPoint = (5.0, 0.0)
        self.eps = 0.5
        self.angEps = 0.0001

        self.minDev, self.minDevPt = 2*math.pi, None
        self.maxDev, self.maxDevPt = 0, None

        self.rate = rospy.Rate(30)

        rospy.Subscriber('/obs_data', ObsData, self.callback_obs)
        rospy.Subscriber('/bot_1/odom', Odometry, self.callback_odom)

        self.pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 10)

        self.startTime = time.time()
        self.timeKeep = []
        self.waypoints = []



    def velocity_convert(self, theta, vel_x, vel_y):
        '''
        Robot pose (x, y, theta)  Note - theta in (0, 2pi)
        Velocity vector (vel_x, vel_y)
        '''

        gain_ang = 1 #modify if necessary
        
        ang = math.atan2(vel_y, vel_x)
        if ang < 0:
            ang += 2 * math.pi
        
        ang_err = min(max(ang - theta, - self.ANG_MAX), self.ANG_MAX)

        v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), - self.VEL_MAX), self.VEL_MAX)
        v_ang = gain_ang * ang_err
        return v_lin, v_ang


    def callback_odom(self, data):
        '''
        Get robot data
        '''
        self.myPose[0] = data.pose.pose.position.x
        self.myPose[1] = data.pose.pose.position.y

        self.myVel[0] = data.twist.twist.linear.x
        self.myVel[1] = data.twist.twist.linear.y
        self.myOmega = data.twist.twist.angular.z

        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        _, _, self.myPose[2] = euler_from_quaternion(quaternion)

        self.timeKeep.append(time.time() - self.startTime)
        self.waypoints.append((self.myPose[0], self.myPose[1]))


    def clearObstacleLists(self):

        self.obsPoseX = np.zeros(3)
        self.obsPoseY = np.zeros(3)
        self.obsPoseVx = np.zeros(3)
        self.obsPoseVy = np.zeros(3)


    def updateObstacleList(self, obstacle, i):

        self.obsPoseX[i] = obstacle.pose_x
        self.obsPoseY[i] = obstacle.pose_y
        self.obsPoseVx[i] = obstacle.vel_x
        self.obsPoseVy[i] = obstacle.vel_y


    def callback_obs(self, data):
        '''
        Get obstacle data in the (pos_x, pos_y, vel_x, vel_y) for each obstacle
        '''

        # self.clearObstacleLists()
        i = 0
        for obstacle in data.obstacles:

            self.updateObstacleList(obstacle, i)
            i += 1


    def dist(self, a, b):
        return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5


    def reachedGoal(self):
        
        return self.dist(self.goalPoint, self.myPose[:2]) < self.eps


    def radialToCartesian(self, vel, ang, origin = (0,0)):

        return (origin[0] + vel*np.cos(ang), origin[1] + vel*np.sin(ang))


    def generateFeasibleSamples(self):
        
        # origin = self.myVel

        theta = self.myPose[2]

        self.samples = []

        for vel in np.linspace(0, self.VEL_MAX, self.VEL_DISCRETE):

            for ang in np.linspace(theta - self.ANG_MAX, theta + self.ANG_MAX, self.ANG_DISCRETE):

                if vel != 0:

                    self.samples.append(self.radialToCartesian(vel, ang))

        # print(len(self.samples))


    def generateCollisionCone(self, i):

        origin = self.myPose[:2]
        obsCentre = (self.obsPoseX[i], self.obsPoseY[i])
        distToObs = ((origin[0]-obsCentre[0])**2 + (origin[1]-obsCentre[1])**2)**0.5
        print(i, obsCentre, distToObs)
        if self.OBS_RADIUS > distToObs:
            return math.pi/2
        # assert self.OBS_RADIUS < distToObs, "Already Collided"
        coneAngle = np.arcsin(self.OBS_RADIUS/distToObs)

        return coneAngle


    def generateAllCollisionCones(self):

        for i in range(len(self.obsPoseX)):
            self.coneAngles[i] = self.generateCollisionCone(i)


    def angleBetweenVectors(self, a, b):
        if not (np.linalg.norm(a) and np.linalg.norm(b)):
            return 0
        return np.arccos(np.dot(a, b)/(np.linalg.norm(a)*np.linalg.norm(b)))

    def collisionWithCone(self, pt, i):
        
        relV = (pt[0] - self.obsPoseVx[i], pt[1] - self.obsPoseVy[i])
        obsVector = (self.obsPoseX[i] - self.myPose[0], self.obsPoseY[i] - self.myPose[1])

        theta = self.angleBetweenVectors(relV, obsVector)
        # print(i)
        return abs(theta) < abs(self.coneAngles[i])


    def inFreeSpace(self, pt):

        for i in range(len(self.obsPoseX)):
            if self.collisionWithCone(pt, i):
                return False

        return True


    def filterPoints(self):

        cones = self.generateAllCollisionCones()

        self.feasiblePts = []

        for pt in self.samples:
            if self.inFreeSpace(pt):
                self.feasiblePts.append(pt)
        # print(len(self.feasiblePts))
        for pt in self.feasiblePts:
            print(pt)


    def findOptimumPoint(self):

        goalDir = (self.goalPoint[0] - self.myPose[0], self.goalPoint[1] - self.myPose[1])

        self.minDev, self.minDevPt = 2*math.pi, None

        for pt in self.feasiblePts:
            currAngle = self.angleBetweenVectors(goalDir, pt)
            if currAngle < self.minDev:
                self.minDev = currAngle
                self.minDevPt = pt


    def takeStep(self, towardsGoal = False):

        v_lin, v_ang = self.velocity_convert(self.myPose[2], self.minDevPt[0], self.minDevPt[1])

        vel_msg = Twist()
        vel_msg.linear.x = v_lin
        vel_msg.angular.z = v_ang
        self.pub_vel.publish(vel_msg)
        # print(v_lin, v_ang)
        self.rate.sleep()


    def avoidObstacles(self):

        while not self.reachedGoal():
                

            self.generateFeasibleSamples() # done

            self.filterPoints() # done

            self.findOptimumPoint() # done

            self.takeStep() # done



def plotData(timeKeep, waypoints):
    wpX = [x[0] for x in waypoints]
    wpY = [x[1] for x in waypoints]

    size = min(len(wpX), len(timeKeep))

    wpX, wpY, timeKeep = wpX[:size], wpY[:size], timeKeep[:size]

    plt.plot(timeKeep, wpX)
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


rospy.init_node('collision_avoidance', anonymous = True, disable_signals = True)

strategy = CollisionAvoidance()
strategy.avoidObstacles()
timeKeep, waypoints = strategy.timeKeep, strategy.waypoints
plotData(timeKeep, waypoints)




