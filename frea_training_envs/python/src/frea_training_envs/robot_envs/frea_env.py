#!/usr/bin/env python3

import copy
import gym
import rospy
import numpy as np
from gym.utils import seeding

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32, Float64
from rosgraph_msgs.msg import Clock
from frea_msgs.msg import Contact
from urdf_parser_py.urdf import URDF

from openai_ros import robot_gazebo_env

class FreaEnv(robot_gazebo_env.RobotGazeboEnv):
    def __init__(self):

        self._contacts = []

        # ==================== SET UP ROS PUBLISHERS ====================
        self.publishers= []

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
            '/adjust_weight/head', Float32, queue_size=1)
        self._tail_weight_pub = rospy.Publisher(
            '/adjust_weight/tail', Float32, queue_size=1)

        rospy.Subscriber('/joint_states', JointState, self.joints_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/contacts', Contact, self.contact_callback)

        # ==================== SET UP FOR SUPER CLASS ====================

        self.controllers_list = [
            '/frea/controllers/state',
            '/frea/controllers/position/left_ear_controller',
            '/frea/controllers/position/right_ear_controller',
            '/frea/controllers/position/mouth_controller',
            '/frea/controllers/position/head_controller',
            '/frea/controllers/position/lower_neck_controller',
            '/frea/controllers/position/upper_neck_controller',
            '/frea/controllers/position/upper_tail_controller',
            '/frea/controllers/position/lower_tail_controller',
            '/frea/controllers/velocity/left_wheel_controller',
            '/frea/controllers/velocity/right_wheel_controller']

        self.robot_name_space = ''
        self.reset_controls = True

        self._seed()
        self.steps_beyond_done = None

        self._urdf = URDF.from_parameter_server('/robot_description')

        super(FreaEnv, self).__init__(
            controllers_list=self.controllers_list,
            robot_name_space=self.robot_name_space,
            reset_controls=self.reset_controls)

        # Unpause gazebo so we can check topics etc.
        self.gazebo.unpauseSim()
        self._check_all_systems_ready()
        self._check_all_publishers_ready()
        self.gazebo.pauseSim()

        rospy.loginfo("Finished FreaEnv INIT")

    def joints_callback(self, data):
        self.joints = data

    def imu_callback(self, data):
        self.imu = data

    def contact_callback(self, data):
        self._contacts.append(data)

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

    def _check_all_publishers_ready(self):
        rate = rospy.Rate(1)
        all_pubs = []
        for pub in self.publishers:
            all_pubs.append(pub)
        all_pubs.append(self._head_weight_pub)
        all_pubs.append(self._tail_weight_pub)
        pub_num = 0
        for pub in all_pubs:
            while(pub.get_num_connections() == 0 and not rospy.is_shutdown()):
                rospy.logerr(
                    "No subscribers to '" + pub.resolved_name +
                    "' yet. Will keep trying.")
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    pass
            rospy.loginfo("'" + pub.resolved_name + "' connected")
            pub_num += 1

        rospy.loginfo("All publishers READY")

    def _check_all_systems_ready(self, init=True):
        self.base_position = None
        self.base_velocity = None
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
                    ok = position_ok and velocity_ok
                    rospy.loginfo("Checking init values Ok => " + str(ok))
            except Exception as e:
                rospy.logerr(e)
                rospy.logerr("'/joint_states' NOT READY yet, will retry.")

        rospy.loginfo("ALL SYSTEMS READY")

    def move_joints(self, joints_array):
        for i in range(len(joints_array)):
            cmd = Float64()
            cmd.data = joints_array[i]
            self.publishers[i].publish(cmd)

    def set_joints(self, joints_array):
        raise NotImplementedError()

    def get_clock_time(self):
        self.clock_time = None
        while self.clock_time is None and not rospy.is_shutdown():
            try:
                self.clock_time = rospy.wait_for_message(
                    "/clock", Clock, timeout=1.0)
                rospy.loginfo("'/clock' READY => " + str(self.clock_time))
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

    def get_contacts(self):
        c = copy.deepcopy(self._contacts)
        self._contacts = []
        return c

    def set_head_mass(self, mass_kg):
        msg = Float32()
        msg.data = mass_kg
        self._head_weight_pub.publish(msg)

    def set_tail_mass(self, mass_kg):
        msg = Float32()
        msg.data = mass_kg
        self._tail_weight_pub.publish(msg)
