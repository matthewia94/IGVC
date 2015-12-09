#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import fuzzy.storage.fcl.Reader
import math
import tf.transformations

class Navigation:
    def __init__(self):
        # Obstacle avoidance
        self.direction_pref = fuzzy.storage.fcl.Reader.Reader().load_from_file('/home/matt/IGVC/ros_ws/src/navigation/scripts/obstacle_direction.fcl')
        self.obstacle_turn = fuzzy.storage.fcl.Reader.Reader().load_from_file('/home/matt/IGVC/ros_ws/src/navigation/scripts/obstacle_avoidance_turn.fcl')
        self.obstacle_linear = fuzzy.storage.fcl.Reader.Reader().load_from_file('/home/matt/IGVC/ros_ws/src/navigation/scripts/obstacle_linear.fcl')
        self.obstacle_weight = fuzzy.storage.fcl.Reader.Reader().load_from_file('/home/matt/IGVC/ros_ws/src/navigation/scripts/obstacle_weight.fcl')

        self.left_dists = {'dist_n': 0.0,
                           'dist_m': 0.0,
                           'dist_f': 0.0}
        self.right_dists = {'dist_n': 0.0,
                            'dist_m': 0.0,
                            'dist_f': 0.0}

        self.preferred_dir_left = {'preferred_dir': 0.0}
        self.preferred_dir_right = {'preferred_dir': 0.0}

        self.input_obstacle_turn = {'dist_f': 0.0,
                                    'pref_dir_l': 0.0,
                                    'pref_dir_r': 0.0}

        self.out_obstacle_turn = {'ang_vel': 0}

        self.obs_dists = {'dist_fl': 0.0,
                          'dist_ml': 0.0,
                          'dist_nl': 0.0,
                          'dist_f': 0.0,
                          'dist_nr': 0.0,
                          'dist_mr': 0.0,
                          'dist_fr': 0.0,
                          'dist_goal': 30.0}

        self.input_obs_linear = {'dist_f': 0.0}
        self.out_obs_linear = {'linear_vel': 0.0}

        self.input_obs_weight = {'dist_f': 0.0}
        self.output_obs_weight = {'weight': 0.0}

        # Goal seeking
        self.goal_y = 0
        self.goal_x = 0
        self.heading = 0
        self.goal_seeking = fuzzy.storage.fcl.Reader.Reader().load_from_file('/home/matt/IGVC/ros_ws/src/navigation/scripts/goal_seeking.fcl')
        self.input_goal = {'heading_error': 0}
        self.output_goal = {'ang_vel': 0}
        self.goal_dist = 0

        # The number of distance fields
        self.num_fields = 7

        # ROS initialization
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        self.pub = rospy.Publisher('/auto_twist', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odomCallback)

        self.twist_msg = Twist()
        self.turn_twist = Twist()
        self.goal_twist = Twist()

        # Training subs and pubs for simulator
        # self.pose_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        # rospy.Subscriber('/base_bumper', ContactsState, self.bumpCallback)

    def laserCallback(self, data):
        # Set the input layer equal to the distances of the lidar normalized
        scan = list(data.ranges[270:810])
        self.obs_dists['dist_fl'] = (min(scan[:(len(scan)/self.num_fields)]))
        self.obs_dists['dist_ml'] = (min(scan[(len(scan)/self.num_fields)+1:2*(len(scan)/self.num_fields)]))
        self.obs_dists['dist_nl'] = (min(2*scan[(len(scan)/self.num_fields)+1:3*(len(scan)/self.num_fields)]))
        self.obs_dists['dist_f'] = (min(3*scan[(len(scan)/self.num_fields)+1:4*(len(scan)/self.num_fields)]))
        self.obs_dists['dist_nr'] = (min(4*scan[(len(scan)/self.num_fields)+1:5*(len(scan)/self.num_fields)]))
        self.obs_dists['dist_mr'] = (min(5*scan[(len(scan)/self.num_fields)+1:6*(len(scan)/self.num_fields)]))
        self.obs_dists['dist_fr'] = (min(6*scan[(len(scan)/self.num_fields)+1:]))

        self.left_dists['dist_n'] = self.obs_dists['dist_nl']
        self.left_dists['dist_m'] = self.obs_dists['dist_ml']
        self.left_dists['dist_f'] = self.obs_dists['dist_fl']
        self.right_dists['dist_n'] = self.obs_dists['dist_nr']
        self.right_dists['dist_m'] = self.obs_dists['dist_mr']
        self.right_dists['dist_f'] = self.obs_dists['dist_fr']

        self.direction_pref.calculate(self.left_dists, self.preferred_dir_left)
        self.direction_pref.calculate(self.right_dists, self.preferred_dir_right)

        self.input_obstacle_turn['dist_f'] = self.obs_dists['dist_f']
        if 0 < self.preferred_dir_left['preferred_dir'] <= 0.33:
            self.input_obstacle_turn['pref_dir_l'] = self.obs_dists['dist_nl']
        elif 0.333 < self.preferred_dir_left['preferred_dir'] <= .66:
            self.input_obstacle_turn['pref_dir_l'] = self.obs_dists['dist_ml']
        else:
            self.input_obstacle_turn['pref_dir_l'] = self.obs_dists['dist_fl']
        if 0 < self.preferred_dir_right['preferred_dir'] <= 0.33:
            self.input_obstacle_turn['pref_dir_r'] = self.obs_dists['dist_nr']
        elif 0.333 < self.preferred_dir_right['preferred_dir'] <= .66:
            self.input_obstacle_turn['pref_dir_r'] = self.obs_dists['dist_mr']
        else:
            self.input_obstacle_turn['pref_dir_r'] = self.obs_dists['dist_fr']
        self.obstacle_turn.calculate(self.input_obstacle_turn, self.out_obstacle_turn)

        # Linear calculation
        self.input_obs_linear['dist_f'] = self.obs_dists['dist_f']
        self.obstacle_linear.calculate(self.input_obs_linear, self.out_obs_linear)

        # Weight calculation
        self.input_obs_weight['dist_f'] = self.obs_dists['dist_f']
        self.obstacle_weight.calculate(self.input_obs_weight, self.output_obs_weight)

        self.turn_twist.linear.x = self.out_obs_linear['linear_vel']
        self.turn_twist.angular.z = self.out_obstacle_turn['ang_vel']

    def update(self):
        if self.obs_dists['dist_f'] > 0.5 and self.goal_dist > .25:
            self.twist_msg.linear.x = 0.5 * self.goal_twist.linear.x + self.output_obs_weight['weight'] * self.turn_twist.linear.x
        else:
            self.twist_msg.linear.x = 0
        self.twist_msg.angular.z = 0.5 * self.goal_twist.angular.z + self.output_obs_weight['weight'] * self.turn_twist.angular.z
        self.pub.publish(self.twist_msg)

    def odomCallback(self, data):
        quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quart)
        self.input_goal['heading_error'] = math.atan2(self.goal_y - data.pose.pose.position.y, self.goal_x - data.pose.pose.position.x) - euler[2]

        self.goal_seeking.calculate(self.input_goal, self.output_goal)

        self.goal_twist.linear.x = 0.5
        self.goal_twist.angular.z = self.output_goal['ang_vel']

        self.goal_dist = math.sqrt(math.pow(data.pose.pose.position.x + self.goal_x, 2) + math.pow(data.pose.pose.position.y + self.goal_y, 2))

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
        # self.pose_pub.publish(s)

if __name__ == '__main__':
    n = Navigation()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        n.update()
        rate.sleep()
