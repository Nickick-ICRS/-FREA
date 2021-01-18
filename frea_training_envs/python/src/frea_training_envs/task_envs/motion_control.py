import numpy as np

import rospy
from tf.transformations import euler_from_quaternion

from gym import spaces

from geometry_msgs.msg import Vector3, Twist

from frea_training_envs.robot_envs.frea_env import FreaEnv
from frea_training_envs.utilities.motion_control_training_rewards \
    import RewardCalculator

MAX_TIMESTEPS = 1e4 # 1e3 timesteps per second -> 10s

class FreaMotionControlEnv(FreaEnv):
    def __init__(self):
        """
        Make FREA learn to follow a requested cmd_vel and hold the head at
        a given position and pitch

        For this task we have the following continuous outputs:
        0: left_wheel_velocity  (-1:1, scaled to the velocity limits)
        1: right_wheel_velocity (-1:1, scaled to the velocity limits)
        2: lower_neck_position  (-1:1, scaled to the joint limits)
        3: upper_neck_position  (-1:1, scaled to the joint limits)
        4: upper_tail_position  (-1:1, scaled to the joint limits)
        5: lower_tail_position  (-1:1, scaled to the joint limits)
        """
        super(FreaMotionControlEnv, self).__init__()

        num_actions = 6

        self.action_space = spaces.Box(
            low=-1,
            high=1,
            shape=(num_actions,),
            dtype=np.float32,
        )

        self.reward_range = (-np.inf, np.inf)

        # Ranges within which valid head positions can be generated
        self.head_min = Vector3()
        self.head_min.x = 0 # Head always >= chassis position
        self.head_min.y = 0 # Can't move Y
        self.head_min.z = 0.05 # Head slightly above ground
        self.head_range = Vector3()
        self.head_range.x = 0.4 # Up to 0.4m forwards
        self.head_range.y = 0 # Can't move Y
        self.head_range.z = 0.65 # Up to 0.7m high

        # Ranges within which valid head pitches can be generated
        self.head_min_p = -1.57/2 # -45 degrees
        self.head_range_p = 1.57*5/6 # Up to +60 degrees

        # Ranges within which valid cmd_vels may be generated
        self.cmd_vel_min_x = -2 # m/s
        self.cmd_vel_min_z = -4 * 3.14 # rad/s
        self.cmd_vel_range_x = -2 * self.cmd_vel_min_x
        self.cmd_vel_range_z = -2 * self.cmd_vel_min_z

        # Ranges for the head and tail masses
        self.head_weight_min = 0 # kg
        self.head_weight_range = 3 # kg
        self.tail_weight_min = 0 # kg
        self.tail_weight_range = 4 # kg

        # Maximum tolerances for the above
        self.max_head_pos_tol = Vector3()
        self.max_head_pos_tol.x = 0.1 # +- 0.1 m
        self.max_head_pos_tol.y = 0.1 # +- 0.1 m
        self.max_head_pos_tol.z = 0.1 # +- 0.1 m
        self.max_head_pitch_tol = 1.57 / 3 # +- 30 degrees
        self.max_cmd_vel_tol_x = 1 # +- 1 m/s
        self.max_cmd_vel_tol_z = 6.28 # +- 6.28 rad/s

        obs_high = np.array([
            # First, joint velocities
            self._urdf.joint_map['left_wheel_joint'].limit.velocity,
            self._urdf.joint_map['right_wheel_joint'].limit.velocity,
            self._urdf.joint_map['lower_neck_joint'].limit.velocity,
            self._urdf.joint_map['upper_neck_joint'].limit.velocity,
            self._urdf.joint_map['upper_tail_joint'].limit.velocity,
            self._urdf.joint_map['lower_tail_joint'].limit.velocity,
            # Second, joint positions
            self._urdf.joint_map['lower_neck_joint'].limit.upper,
            self._urdf.joint_map['upper_neck_joint'].limit.upper,
            self._urdf.joint_map['upper_tail_joint'].limit.upper,
            self._urdf.joint_map['lower_tail_joint'].limit.upper,
            # Third, IMU pitch
            1.57,
            # Fourth, cmd_vel limits
            self.cmd_vel_min_x + self.cmd_vel_range_x,
            self.cmd_vel_min_z + self.cmd_vel_range_z,
            # Fifth, head pose limits
            self.head_min.x + self.head_range.x,
            self.head_min.y + self.head_range.y,
            self.head_min.z + self.head_range.z,
            self.head_min_p + self.head_range_p,
            # Sixth, tolerances
            self.max_head_pos_tol.x,
            self.max_head_pos_tol.y,
            self.max_head_pos_tol.z,
            self.max_head_pitch_tol,
            self.max_cmd_vel_tol_x,
            self.max_cmd_vel_tol_z
        ])

        obs_low = np.array([
            # First, joint velocities
            -self._urdf.joint_map['left_wheel_joint'].limit.velocity,
            -self._urdf.joint_map['right_wheel_joint'].limit.velocity,
            -self._urdf.joint_map['lower_neck_joint'].limit.velocity,
            -self._urdf.joint_map['upper_neck_joint'].limit.velocity,
            -self._urdf.joint_map['upper_tail_joint'].limit.velocity,
            -self._urdf.joint_map['lower_tail_joint'].limit.velocity,
            # Second, joint positions
            self._urdf.joint_map['lower_neck_joint'].limit.lower,
            self._urdf.joint_map['lower_neck_joint'].limit.lower,
            self._urdf.joint_map['lower_tail_joint'].limit.lower,
            self._urdf.joint_map['lower_tail_joint'].limit.lower,
            # Third, IMU pitch
            -1.57,
            # Fourth, cmd_vel limits
            self.cmd_vel_min_x,
            self.cmd_vel_min_z,
            # Fifth, head pose limits
            self.head_min.x,
            self.head_min.y,
            self.head_min.z,
            self.head_min_p,
            # Sixth, tolerances
            0,
            0,
            0,
            0,
            0,
            0
        ])

        self.observation_space = spaces.Box(obs_low, obs_high)

        rospy.logdebug("ACTION SPACE TYPE ==> " + str(self.action_space))
        rospy.logdebug(
            "OBSERVATION SPACE TYPE ==> " + str(self.observation_space))

        self._rc = RewardCalculator()

    def prepare_for_next_iteration(self):
        self.target_head_position, self.head_pos_tol = \
            self.generate_head_position()
        self.target_head_pitch, self.head_pitch_tol = \
            self.generate_head_pitch()
        self.target_cmd_vel, self.cmd_vel_tol = self.generate_cmd_vel()
        rospy.loginfo("Target cmd_vel:\n" + str(self.target_cmd_vel))
        rospy.loginfo("Target head_pos:\n" + str(self.target_head_position))
        rospy.loginfo("Target head_pitch:\n" + str(self.target_head_pitch))
        self.generate_random_head_and_tail_weights()

    def generate_head_position(self):
        vec = Vector3()
        tol = Vector3()
        r = np.random.rand(6)
        vec.x = self.head_min.x + r[0] * self.head_range.x
        vec.y = self.head_min.y + r[1] * self.head_range.y
        vec.z = self.head_min.z + r[2] * self.head_range.z
        tol.x = r[3] * self.max_head_pos_tol.x
        tol.y = r[4] * self.max_head_pos_tol.y
        tol.z = r[5] * self.max_head_pos_tol.z
        return vec, tol

    def generate_head_pitch(self):
        r = np.random.rand(2)
        p = self.head_min_p + r[0] * self.head_range_p
        tol = r[1] * self.max_head_pitch_tol
        return p, tol

    def generate_cmd_vel(self):
        cmd = Twist()
        tol = Twist()
        r = np.random.rand(4)
        cmd.linear.x = self.cmd_vel_min_x + r[0] * self.cmd_vel_range_x
        cmd.angular.z = self.cmd_vel_min_z + r[1] * self.cmd_vel_range_z
        tol.linear.x = r[0] * self.max_cmd_vel_tol_x
        tol.angular.z = r[1] * self.max_cmd_vel_tol_z
        return cmd, tol

    def generate_random_head_and_tail_weights(self):
        r = np.random.rand(2)
        head = self.head_weight_min + r[0] * self.head_weight_range
        tail = self.tail_weight_min + r[1] * self.tail_weight_range

        rospy.loginfo("Setting head mass to {} kg".format(head))
        rospy.loginfo("Setting tail mass to {} kg".format(tail))

        self.set_head_mass(head)
        self.set_tail_mass(tail)

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        pass

    def _init_env_variables(self):
        self.cumulated_reward = 0
        self.timestep = 0
        self.prepare_for_next_iteration()

    def _set_action(self, action):
        rospy.logdebug("Action given ==> " + str(action))

        # Force actions to be in the range 0 - 1
        for i in range(len(action)):
            bounded_act = min(max(action[0], -1), 1)
            if bounded_act != action[i]:
                rospy.logdebug(
                    "Input action not in action bounds (" + 
                    str(action[i]) + "!=" + str(bounded_act) + ").")
                action[i] = bounded_act
            # Now that limits are bounded to -1 and 1, switch to 0-1
            action[i] = action[i]/2 + 0.5

        # We care about 0, 1, 2, 3, 4, 5, but 6, 7, 8, 9 need to be set to 0
        joint_values = np.zeros(10)
        joint_values[0] = ( # left wheel
            -self._urdf.joint_map['left_wheel_joint'].limit.velocity +
            2 * action[0] * 
            self._urdf.joint_map['left_wheel_joint'].limit.velocity)
        joint_values[1] = ( # right wheel
            -self._urdf.joint_map['right_wheel_joint'].limit.velocity +
            2 * action[1] * 
            self._urdf.joint_map['right_wheel_joint'].limit.velocity)
        joint_values[2] = ( # lower neck
            self._urdf.joint_map['lower_neck_joint'].limit.lower +
            action[2] *
            (self._urdf.joint_map['lower_neck_joint'].limit.upper
            - self._urdf.joint_map['lower_neck_joint'].limit.lower))
        joint_values[3] = ( # upper neck
            self._urdf.joint_map['upper_neck_joint'].limit.lower +
            action[3] *
            (self._urdf.joint_map['upper_neck_joint'].limit.upper
            - self._urdf.joint_map['upper_neck_joint'].limit.lower))
        joint_values[4] = ( # upper tail
            self._urdf.joint_map['upper_tail_joint'].limit.lower +
            action[4] *
            (self._urdf.joint_map['upper_tail_joint'].limit.upper
            - self._urdf.joint_map['upper_tail_joint'].limit.lower))
        joint_values[5] = ( # lower tail
            self._urdf.joint_map['lower_tail_joint'].limit.lower +
            action[5] *
            (self._urdf.joint_map['lower_tail_joint'].limit.upper
            - self._urdf.joint_map['lower_tail_joint'].limit.lower))
        joint_values[6] = 0 # head
        joint_values[7] = 0 # mouth
        joint_values[8] = 0 # left ear
        joint_values[9] = 0 # right ear
    
        rospy.logdebug("Executing action ==> " + str(joint_values))
        self.move_joints(joint_values)

    def _get_obs(self):
        # Get the observations we care about
        self.timestep += 1
        
        js = self.get_joints()
        imu = self.get_imu()
        
        quat = (
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w)

        imu_pitch = euler_from_quaternion(quat)[1]

        obs = np.array([
            # First, joint velocities
            self.get_joint_vel(js, 'left_wheel_joint'),
            self.get_joint_vel(js, 'right_wheel_joint'),
            self.get_joint_vel(js, 'lower_neck_joint'),
            self.get_joint_vel(js, 'upper_neck_joint'),
            self.get_joint_vel(js, 'upper_tail_joint'),
            self.get_joint_vel(js, 'lower_tail_joint'),
            # Second, joint positions
            self.get_joint_pos(js, 'lower_neck_joint'),
            self.get_joint_pos(js, 'upper_neck_joint'),
            self.get_joint_pos(js, 'upper_tail_joint'),
            self.get_joint_pos(js, 'lower_tail_joint'),
            # Third, IMU pitch
            imu_pitch,
            # Fourth, cmd_vel
            self.target_cmd_vel.linear.x,
            self.target_cmd_vel.angular.z,
            # Fifth, head pose
            self.target_head_position.x,
            self.target_head_position.y,
            self.target_head_position.z,
            self.target_head_pitch,
            # Sixth, tolerances
            self.head_pos_tol.x,
            self.head_pos_tol.y,
            self.head_pos_tol.z,
            self.head_pitch_tol,
            self.cmd_vel_tol.linear.x,
            self.cmd_vel_tol.angular.z
        ])

        return obs

    def _is_done(self, observations):
        """
        We're done if the robot has fallen over or the max timesteps has
        been reached
        """
        if self.timestep >= MAX_TIMESTEPS:
            return True
        if len(self.get_contacts()):
            return True
        return False

    def _compute_reward(self, observations, done):
        rew = self._rc.calculate_reward(
            self.target_cmd_vel, self.cmd_vel_tol,
            self.target_head_position, self.head_pos_tol,
            self.target_head_pitch, self.head_pitch_tol)

        rospy.logdebug("################")
        rospy.logdebug("Reward: " + str(rew))
        rospy.logdebug("################")

        return rew

    def get_joint_vel(self, joint_state, joint):
        for i in range(len(joint_state.name)):
            if joint_state.name[i] == joint:
                return joint_state.velocity[i]

    def get_joint_pos(self, joint_state, joint):
        for i in range(len(joint_state.name)):
            if joint_state.name[i] == joint:
                return joint_state.position[i]
