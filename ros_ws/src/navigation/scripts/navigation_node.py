#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from NeuralNetwork import NeuralNetwork
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Pose
import numpy as np
import time


class Navigation:
    def __init__(self):
        self.network = NeuralNetwork(540, 541, 2)

        # ROS initialization
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        self.pub = rospy.Publisher('/auto_twist', Twist, queue_size=1)

        # Training subs and pubs for simulator
        self.pose_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        rospy.Subscriber('/base_bumper', ContactsState, self.bumpCallback)

        self.twist_msg = Twist()

    def laserCallback(self, data):
        # Set the input layer equal to the distances of the lidar normalized
        self.network.input_layer = np.array(data.ranges[0::2]) / data.range_max
        self.network.feedforward()
        self.network.output_layer[0] = self.network.output_layer[0]/max(self.network.output_layer)
        self.network.output_layer[1] = self.network.output_layer[1]/max(self.network.output_layer)
        self.network.backpropagation(self.error())

    def update(self):
        self.twist_msg.linear.x = self.network.output_layer[0]/max(self.network.output_layer)
        self.twist_msg.angular.z = self.network.output_layer[1]/max(self.network.output_layer)
        self.pub.publish(self.twist_msg)
        print self.network.output_layer

    def bumpCallback(self, data):
        if len(data.states) > 0:
            self.resetRobot()rest

    def resetRobot(self):
        s = ModelState()
        s.model_name = 'igvc'
        p = Pose()
        p.position.x = -2
        p.position.y = -21.5
        s.pose = p
        self.pose_pub.publish(s)

    def error(self):
        return [min(1/self.network.input_layer)] * self.network.num_outputs

if __name__ == '__main__':
    n = Navigation()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        n.update()
        rate.sleep()
