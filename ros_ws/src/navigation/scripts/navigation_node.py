#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from NeuralNetwork import NeuralNetwork


class Navigation:
    def __init__(self):
        self.dists = list()

        # ROS initialization
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        self.pub = rospy.Publisher('/auto_twist', Twist, queue_size=1)

        self.twist_msg = Twist()

        self.net = NeuralNetwork(1080, 1100, 2)

    def laserCallback(self, data):
        self.dists = data.ranges

    def update(self):
        self.twist_msg.angular.z = 1
        self.pub.publish(self.twist_msg)

if __name__ == '__main__':
    n = Navigation()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        n.update()
        rate.sleep()
