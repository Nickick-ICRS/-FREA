#!/usr/bin/env python

import gym
import rospy
import numpy as np
from gym.utils import seeding

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from urdf_parser_py.urdf import URDF

from openai_ros import robot_gazebo_env

class FreaEnv(robot_gazebo_env.RobotGazeboEnv):
    def __init__(self):

        # ==================== SET UP ROS PUBLISHERS ====================
        self.publishers_array = []

        self._left_wheel_pub = rospy.Publisher(
            '/frea/controllers/velocity/left_wheel_controller/command',
            Float64, queue_size=1)
        self._right_wheel_pub = rospy.Publisher(
            '/frea/controllers/velocity/right_wheel_controller/command',
            Float64, queue_size=1)
        self._lower_neck_pub = rospy.Publisher(
            'frea/controllers/position/lower_neck_controller/command',
            Float64, queue_size=1)
        self._upper_neck_pub = rospy.Publisher(
            'frea/controllers/position/upper_neck_controller/command',
            Float64, queue_size=1)
        self._upper_tail_pub = rospy.Publisher(
            'frea/controllers/position/upper_tail_controller/command',
            Float64, queue_size=1)
        self._lower_tail_pub = rospy.Publisher(
            'frea/controllers/position/lower_tail_controller/command',
            Float64, queue_size=1)
        self._head_pub = rospy.Publisher(
            'frea/controllers/position/head_controller/command',
            Float64, queue_size=1)
        self._mouth_pub = rospy.Publisher(
            'frea/controllers/position/mouth_controller/command',
            Float64, queue_size=1)
        self._left_ear_pub = rospy.Publisher(
            'frea/controllers/position/left_ear_controller/command',
            Float64, queue_size=1)
        self._right_ear_pub = rospy.Publisher(
            'frea/controllers/position/right_ear_controller/command',
            Float64, queue_size=1)

        self.publishers.append(self._left_wheel_pub)
        self.publishers.append(self._right_wheel_pub)
        self.publishers.append(self._lower_neck_pub)
        self.publishers.append(self._upper_neck_pub)
        self.publishers.append(self._upper_tail_pub)
        self.publishers.append(self._lower_tail_pub)
        self.publishers.append(self._head_pub)
        self.publishers.append(self._mouth_pub)
        self.publishers.append(self._left_ear_pub)
        self.publishers.append(self._right_ear_pub)

        self._head_weight_pub = rospy.Publisher(
            'adjust_weight/head', Float64, queue_size=1)
        self._tail_weight_pub = rospy.Publisher(
            'adjust_weight/tail', Float64, queue_size=1)

        rospy.Subscriber('/joint_states', JointState, self.joints_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # ==================== SET UP FOR SUPER CLASS ====================

        self.controllers_list = [
            'joint_state_controller',
            'left_ear_controller',
            'right_ear_controller',
            'mouth_controller',
            'head_controller',
            'lower_neck_controller',
            'upper_neck_controller',
            'upper_tail_controller',
            'lower_tail_controller',
            'left_wheel_controller',
            'right_wheel_controller']

        self.robot_name_space = ''
        self.reset_controls = True

        self._seed()
        self.steps_beyond_done = None

        self._urdf = URDF.from_parameter_server('/robot_description')

        super(FreaEnv, self).__init__(
            controllers_list=self.controllers_list,
            robot_names_space=self.robot_name_space,
            reset_controls=self.reset_controls)

        # Unpause gazebo so we can check topics etc.
        self.gazebo.unpauseSim()
        self._check_all_systems_ready()
        self._check_all_publishers_read()
        self.gazebo.pauseSim()

        rospy.loginfo("Finished FreaEnv INIT")

    def joints_callback(self, data):
        self.joints = data

    def imu_callback(self, data):
        self.imu = data

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
            

    # RobotEnv methods

    def _env_setup(self, initial_qpos):
        self.init_internal_vars()
        self._set_init_pose()
        self._check_all_systems_ready()

    def init_internal_vars(self):
        self.joints = None
        self.imu = None

    def check_publishers_connection(self):
        rate = rospy.Rate(10)
        all_pubs = []
        for pub in self.publishers:
            all_pubs.append(pub)
        all_pubs.append(self._head_weight_pub)
        all_pubs.append(self._tail_weight_pub)
        for pub in all_pubs:
            topic_name = pub.get_topic_name()
            while(pub.get_num_connections() == 0 and not rospy.is_shutdown()):
                rospy.logdebug(
                    "No subscribers to '" + topic_name +
                    "' yet. Will keep trying.")
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    pass
            rospy.logdebug("'" + topic_name + "' connected")

        rospy.logdebug("All publishers READY")

    def _check_all_systems_ready(self, init=True):
        self.base_position = None
        while self.base_position is None and not rospy.is_shutdown():
            try:
                self.base_position = rospy.wait_for_message(
                    "/joint_states", JointState, timeout=1.0)
                rospy.logdebug(
                    "'/joint_states' READY => "
                    + str(self.base_position))
                if init:
                    # Check sensors are at initial values
                    position_ok = all(
                        abs(i) <= 1e-2 for i in self.base_position.position)
                    velocity_ok = all(
                        abs(i) <= 1e-2 for i in self.base_velocity.velocity)
                    efforts_ok = all(
                        abs(i) <= 1e-2 for i in self.base_effort.effort)
                    ok = position_ok and velocity_ok and effort_ok
                    rospy.logdebug("Checking init values Ok => " + str(ok))
            except:
                rospy.logerr("'/joint_states' NOT READY yet, will retry.")

        rospy.logdebug("ALL SYSTEMS READY")

    def move_joints(self, joints_array):
        for i in range(len(joints_array)):
            cmd = Float64()
            cmd.data = joints_array[i]
            self.publishers[i].publish(cmd)

    def set_joints(self, joints_array):
        for i in range(joints_array):
            

    def get_clock_time(self):
        self.clock_time = None
        while self.clock_time is None and not rospy.is_shutdown():
            try:
                self.clock_time = rospy.wait_for_message(
                    "/clock", Clock, timeout=1.0)
                rospy.logdebug("'/clock' READY => " + str(self.clock_time))
            except:
                rospy.logerr("Waiting for '/clock' to come online")
        return self.clock_time

    # Methods that the TrainingEnvironment will need to define
    # because they will be used in RobotGazeboEnv GrandParentClass
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()

    def get_joints(self):
        return self.joints

    def get_imu(self):
        return self.imu

    def set_head_mass(self, mass_kg):
        msg = Float64()
        msg.data = mass_kg
        self._head_weight_pub.publish(msg)

    def set_head_mass(self, mass_kg):
        msg = Float64()
        msg.data = mass_kg
        self._tail_weight_pub.publish(msg)
