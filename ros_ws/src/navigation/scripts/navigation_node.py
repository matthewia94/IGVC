#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Pose
import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt

class Navigation:
    def __init__(self):
        self.dists = list()

        # The number of distance fields
        self.num_fields = 5
        self.dists_obstacles = np.arange(0, 30, 1)
        self.goal_dist = np.arange(0, 100, 1)

        # Membership functions
        self.goal_dist_far = fuzz.trapmf(self.goal_dist, [7, 10, 100, 100])
        self.goal_dist_close = fuzz.trapmf(self.goal_dist, [0, 0, 10, 12])
        self.obstacle_dist_very_close = fuzz.trapmf(self.dists_obstacles, [0, 0, 6, 7])
        self.obstacle_dist_close = fuzz.trapmf(self.dists_obstacles, [5, 6, 12, 13])
        self.obstacle_dist_moderate = fuzz.trapmf(self.dists_obstacles, [11, 12, 18, 19])
        self.obstacle_dist_far = fuzz.trapmf(self.dists_obstacles, [17, 18, 24, 25])
        self.obstacle_dist_very_far = fuzz.trapmf(self.dists_obstacles, [23, 24, 30, 30])

        self.display_members()

        # ROS initialization
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        self.pub = rospy.Publisher('/auto_twist', Twist, queue_size=1)

        # Training subs and pubs for simulator
        self.pose_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        rospy.Subscriber('/base_bumper', ContactsState, self.bumpCallback)

        self.twist_msg = Twist()

    def display_members(self):
        fig, (mem1, mem2) = plt.subplots(nrows=2)
        mem1.plot(self.goal_dist, self.goal_dist_far, 'b', linewidth=2, label='Far')
        mem1.plot(self.goal_dist, self.goal_dist_close, 'g', linewidth=2, label='Close')
        mem1.legend()
        mem1.spines['top'].set_visible(False)
        mem1.spines['right'].set_visible(False)
        mem1.get_xaxis().tick_bottom()
        mem1.get_yaxis().tick_left()

        mem2.plot(self.dists_obstacles, self.obstacle_dist_very_close, 'b', linewidth=2, label='Very Close')
        mem2.plot(self.dists_obstacles, self.obstacle_dist_close, 'g', linewidth=2, label='Close')
        mem2.plot(self.dists_obstacles, self.obstacle_dist_moderate, 'r', linewidth=2, label='Moderate')
        mem2.plot(self.dists_obstacles, self.obstacle_dist_far, 'm', linewidth=2, label='Far')
        mem2.plot(self.dists_obstacles, self.obstacle_dist_very_far, 'y', linewidth=2, label='Very Far')
        mem2.legend()
        mem2.spines['top'].set_visible(False)
        mem2.spines['right'].set_visible(False)
        mem2.get_xaxis().tick_bottom()
        mem2.get_yaxis().tick_left()
        plt.tight_layout()
        plt.show()

    def laserCallback(self, data):
        # Set the input layer equal to the distances of the lidar normalized
        self.dists = []
        scan = list(data.ranges[270:810])
        for i in range(self.num_fields):
            self.dists.append(min(scan[i*(len(scan)/self.num_fields):(i+1)*(len(scan)/self.num_fields)]))

    def update(self):
        self.twist_msg.linear.x = 0
        self.twist_msg.angular.z = 0
        self.pub.publish(self.twist_msg)

    def bumpCallback(self, data):
        if len(data.states) > 0:
            self.resetRobot()

    def resetRobot(self):
        s = ModelState()
        s.model_name = 'igvc'
        p = Pose()
        p.position.x = -2
        p.position.y = -21.5
        s.pose = p
        self.pose_pub.publish(s)

if __name__ == '__main__':
    n = Navigation()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        n.update()
        rate.sleep()
