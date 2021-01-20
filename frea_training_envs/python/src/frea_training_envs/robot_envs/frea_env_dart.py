#!/usr/bin/env python

import os
import copy
import gym
import rospy
import rospkg
import tf
import numpy as np
from gym.utils import seeding

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32, Float64
from rosgraph_msgs.msg import Clock
from frea_msgs.msg import Contact
from gazebo_msgs.msg import LinkState
from urdf_parser_py.urdf import URDF

import dartpy as dart

class FreaEnv(gym.Env):
    def __init__(self):
        gym.Env.__init__(self)
        
        world_path = os.path.join(
            rospkg.RosPack().get_path("frea_training_envs"),
            "worlds/flat_plane.skel")

        self.world = dart.utils.SkelParser.readWorld(world_path)

        urdf_path = os.path.join(
            rospkg.RosPack().get_path("frea_description"),
            "urdf/generated/frea_dart.urdf")

        skeleton = dart.utils.DartLoader().parseSkeleton(urdf_path)

        self.world.addSkeleton(skeleton)

        self.world.setGravity([0, 0, -9.8])
        self.world.setTimeStep(0.01)

        self.collision_detector = \
            self.world.getConstraintSolver().getCollisionDetector()

        self.frea = self.world.getSkeleton('frea')
        self.frea.setPosition(2, 0.1)
        self.frea_joints = self.frea.getJoints()

        self._contacts = []

        self._seed()
        self.steps_beyond_done = None

        self._urdf = URDF.from_parameter_server('/robot_description')

        self._joint_state_publisher = rospy.Publisher(
            '/joint_states', JointState, queue_size=2)

        self._clock_publisher = rospy.Publisher(
            '/clock', Clock, queue_size=1)

        self._contact_publisher = rospy.Publisher(
            '/contacts', Contact, queue_size=100)

        self.setup_visualisation()

        rospy.loginfo("Finished FreaEnv INIT")

    def setup_visualisation(self):
        node = dart.gui.osg.WorldNode(self.world)

        self.viewer = dart.gui.glut.SimWindow() #dart.gui.osg.Viewer()
        self.viewer.setWorld(self.world)
        glutInit()
        self.viewer.initWindow(640, 480, "frea")
        #self.viewer.addWorldNode(node)

        #grid = dart.gui.osg.GridVisual()
        #grid.setPlaneType(dart.gui.osg.GridVisual.PlaneType.XY)
        #grid.setOffset([0, 0, 0])
        #self.viewer.addAttachment(grid)

        #self.viewer.setUpViewInWindow(0, 0, 640, 480)
        #self.viewer.setCameraHomePosition([8, -8, 4],
        #                                  [0, 0, -0.25],
        #                                  [0, 0, 0.5])
        
        self._frame_cnt = 0
        self._frame_ticks = 15

    def visualise(self):
        if self._frame_cnt % self._frame_ticks == 0:
            #self.viewer.frame() Need to wrap the C++ code first...
            self._frame_cnt = 0
        self._frame_cnt += 1

    def reset(self):
        self._simulation_reset()
        self._set_init_pose()
        self._init_env_variables()
        self._update_sensor_readings()
        self._publish_sensor_readings()
        return self._get_obs()

    def step(self, action):
        self.timestep += 1
        self._set_action(action)
        self.world.step()
        self._update_sensor_readings()
        self._publish_sensor_readings()
        obs = self._get_obs()
        done = self._is_done(obs)
        reward = self._compute_reward(obs, done)
        info = {}

        self.visualise()

        return obs, reward, done, info

    def _read_joint_states(self):

        names = []
        positions = []
        velocities = []
        efforts = []

        for joint in self.frea_joints:
            if joint.getNumDofs() and joint.getName() != "world_frea":
                names.append(joint.getName())
                positions.append(joint.getPosition(0))
                velocities.append(joint.getVelocity(0))
                efforts.append(joint.getForce(0))

        self.joints = JointState()
        self.joints.name = names
        self.joints.position = positions
        self.joints.velocity = velocities
        self.joints.effort = efforts

    def _read_contacts(self):
        self._contacts = []
        res = self.collision_detector.getLastCollisionResult()
        contacts = res.getContacts()
        for contact in contacts:
            o1 = contact.collisionObject1
            o2 = contact.collisionObject2
            if (contact.bodyNode1.lock().getSkeleton() == self.frea or
                contact.bodyNode2.lock().getSkeleton() == self.frea):
                c = Contact()
                c.object1 = ""
                c.object2 = ""
                self._contacts.append(c)

    def _publish_sensor_readings(self):
        clock = Clock()
        clock.clock = rospy.Time(self.world.getTimeStep() * self.timestep)
        self._clock_publisher.publish(clock)

        self.joints.header.stamp = clock.clock
        self._joint_state_publisher.publish(self.joints)

        for contact in self._contacts:
            self._contact_publisher.publish(contact)

    def _update_sensor_readings(self):
        self._read_joint_states()
        self._read_contacts()

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

    def _simulation_reset(self):
        self.world.reset()
        self.timestep = 0

        self.frea.resetPositions()
        self.frea.resetVelocities()
        self.frea.resetAccelerations()

    def move_joints(self, joints_array):
        for i in range(len(joints_array)):
            cmd = Float64()
            cmd.data = joints_array[i]

    def set_joints(self, joints_array):
        raise NotImplementedError()

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

    def get_contacts(self):
        return self._contacts

    def set_head_mass(self, mass_kg):
        self.frea.getBodyNode("head_weight_link").setMass(mass_kg)

    def set_tail_mass(self, mass_kg):
        self.frea.getBodyNode("tail_weight_link").setMass(mass_kg)

    def get_exact_body_state(self, body_name):
        body = self.frea.getBodyNode(body_name)
        trans = body.getWorldTransform()
        lin = body.getLinearVelocity()
        ang = body.getAngularVelocity()
        state = LinkState()
        state.pose.position.x = trans.translation()[0]
        state.pose.position.y = trans.translation()[1]
        state.pose.position.z = trans.translation()[2]
        R = np.zeros((4, 4))
        R[0:3, 0:3] = trans.rotation()
        R[3, 3] = 1
        q = tf.transformations.quaternion_from_matrix(R)
        state.pose.orientation.x = q[0]
        state.pose.orientation.y = q[1]
        state.pose.orientation.z = q[2]
        state.pose.orientation.w = q[3]
        state.twist.linear.x = lin[0]
        state.twist.linear.y = lin[1]
        state.twist.linear.z = lin[2]
        state.twist.angular.x = ang[0]
        state.twist.angular.y = ang[1]
        state.twist.angular.z = ang[2]
        return state
