import rospy
import gym
import time
from gym.utils import seeding

from .dart_connection import DartConnection

from openai_ros.controllers_connection import ControllersConnection
from openai_ros.msg import RLExperimentInfo

class DartEnv(gym.Env):
    
    def __init__(self, robot_name_space, controllers_list, reset_controls):
        self.simulator = DartConnection()
        self.controllers_object = ControllersConnection(
            namespace=robot_name_space, controllers_list=controllers_list)
        self.reset_controls = reset_controls
        self.seed()

        self.episode_num = 0
        self.cumulated_episode_reward = 0
        self.reward_pub = rospy.Publisher(
            '/openai/reward', RLExperimentInfo, queue_size=1)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        self._set_action(action)
        self.simulator.step()
        obs = self._get_obs()
        done = self._is_done(obs)
        info = {}
        reward = self._compute_reward(obs, done)
        self.cumulated_episode_reward += reward

        return obs, reward, done, info

    def reset(self):
        self._reset_sim()
        self._init_env_variables()
        self._update_episode()
        obs = self._get_obs()
        return obs

    def close(self):
        rospy.signal_shutdown("Closing Dart Environment")
        
    def _update_episode(self):
        self._publish_reward_topic(
            self.cumulated_episode_reward, self.episode_num)
        self.episode_num += 1
        self.cumulated_episode_reward = 0

    def _publish_reward_topic(self, reward, episode_number=1):
        reward_msg = RLExperimentInfo()
        reward_msg.episode_number = episode_number
        reward_msg.episode_reward = reward
        self.reward_pub.publish(reward_msg)

    def _reset_sim(self):
        if self.reset_controls:
            self.simulator.resetSim()
            self._set_init_pose()
            self.simulator.unpauseSim()
            self.controllers_object.reset_controllers()
            self._check_all_systems_ready()
            self.simulator.pauseSim()
        
        else:
            self.simulator.resetSim()
            self._set_init_pose()
            self.simulator.unpauseSim()
            self._check_all_systems_ready()
            self.simulator.pauseSim()

        return True
    def _set_init_pose(self):
        raise NotImplementedError()
    
    def _check_all_systems_ready(self):
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _init_env_variables(self):
        raise NotImplementedError()

    def _set_action(self, action):
        raise NotImplementedError()

    def _is_done(self, observations):
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        raise NotImplementedError()

    def _env_setup(self, initial_qpos):
        raise NotImplementedError()
