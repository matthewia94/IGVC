#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from NeuralNetwork import NeuralNetwork
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Pose


class Navigation:
    def __init__(self):
        self.network = NeuralNetwork(540, 550, 2)

        # ROS initialization
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        self.pub = rospy.Publisher('/auto_twist', Twist, queue_size=1)

        # Training subs and pubs for simulator
        self.pose_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        rospy.Subscriber('/base_bumper', ContactsState, self.bumpCallback)

        self.twist_msg = Twist()

    def laserCallback(self, data):
        # Set the input layer equal to the distances of the lidar
        self.network.input_layer = data.ranges[0::2]

    def update(self):
        self.twist_msg.angular.z = 1
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
