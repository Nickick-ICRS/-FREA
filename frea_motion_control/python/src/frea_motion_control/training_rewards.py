import math

import rospy
from tf.transformations import euler_from_quaternion

from gazebo_msgs.srv import GetLinkState
from sensor_msgs.msg import JointState

class RewardCalculator:
    def __init__(self):
        # Service to ask gazebo for the exact position of a link wrt another
        rospy.wait_for_service('/gazebo/get_link_state')
        self._get_link_state = rospy.ServiceProxy(
            '/gazebo/get_link_state', GetLinkState)

        rospy.Subscriber(
            '/joint_states', JointState, self.joint_state_callback)

        # Calculated everytime we receive a joint state message
        self._energy_reward = 0

        # =========== Reward parameters ===========
        # Linear slider for whether we prioritise saving energy or
        # accurate movement 0 = ignore accuracy, 1 = ignore energy
        self._alpha = 0.95
        # Multiplier to scale the energy reward
        self._energy_mult = 1
        # Precision multiplier for position penalty
        # e.g. 0.01: cm precision = 0.5x reward, mm precision = 0.91x reward
        self._position_penalty = 0.01


    def calculate_reward(
        self, target_twist, twist_tolerance, target_head_position,
        head_position_tolerance, target_head_pitch, head_pitch_tolerance):
        """
        @brief Calculates the reward based on the requested values,
               tolerances and the true values

        @param target_twist geometry_msgs/Twist The required cmd_vel of the
                            base_link. Only linear X and angular Z are
                            considered

        @param twist_tolerance geometry_msgs/Twist The tolerance required of
                               the cmd_vel. Only linear X and angular Z are
                               considered

        @param target_head_position geometry_msgs/Vector3 The required
                                    position of the head wrt the base_link

        @param head_position_tolerance geometry_msgs/Vector3 The required
                                       tolerance of the head position

        @param target_head_pitch float The required pitch of the head
                                 in the base_link frame
 
        @param head_pitch_tolerance float The required tolerance of the head
                                    pitch
 
        @returns (float) reward of how well the current values meet the
                 required values and their tolerances
        """

        vx, vz, hx, hy, hz, hp = self.calculate_errors(
            target_twist, target_head_position, target_head_pitch)

        # Subtract the tolerances to find the true error
        vx = min(vx - twist_tolerance.linear.x, 0)
        vz = min(vz - twist_tolerance.angular.z, 0)
        hx = min(hx - head_position_tolerance.x, 0)
        hy = min(hy - head_position_tolerance.y, 0)
        hz = min(hz - head_position_tolerance.z, 0)
        hp = min(hp - head_pitch_tolerance, 0)

        rmse = math.sqrt(
            vx ** 2 + vz ** 2 + hx ** 2 + hy ** 2 + hz ** 2 + hp ** 2)
        state_reward = self._position_penalty / \
            (rmse + self._position_penalty)

        total_reward = self._alpha * state_reward + \
            (1 - self._alpha) * self._energy_reward

        return total_reward


    def calculate_errors(
        self, target_twist, target_head_position, target_head_pitch):
        """
        @brief Calculates the error betweem the requested values and the
               true values

        @param target_twist geometry_msgs/Twist The required cmd_vel of the
                            base_link

        @param target_head_position geometry_msgs/Vector3 The required
                                    position of the head wrt the base_link

        @param target_head_pitch float The required pitch of the head
                                 in the base_link frame

        @returns The (absolute) errors in the following order:
                 twist linear x
                 twist angular z
                 head position x
                 head position y
                 head position z
                 head pitch
        """

        # Read the state of the chassis and head
        chassis_state = self._get_link_state('chassis_link', 'world')
        head_state = self._get_link_state('head_link', 'world')

        # We only care about yaw velocity and base_link frame x velocity
        # Yaw is unchanged in the world frame, and x is just the normal
        # of x and y in the world frame
        chassis_z_vel = chassis_state.twist.angular.z
        chassis_x_vel = math.sqrt(
            chassis_state.twist.linear.x ** 2,
            chassis_state.twist.linear.y ** 2)

        # Calculate errors
        twist_x_err = abs(chassis_x_vel - target_twist.linear.x)
        twist_z_err = abs(chassis_z_vel - target_twist.angular.z)
        

        # Find head position wrt base link
        # Base link isn't in gazebo, so we read both wrt the origin and then
        # subtract the chassis position from the head position
        head_pos = head_state.pose.position
        head_pos.x -= chassis_state.pose.position.x
        head_pos.y -= chassis_state.pose.position.y
        head_pos.z -= chassis_state.pose.position.z

        # Extract the pitch of the head. In the world frame this is
        # identical to the base_link frame
        quat = (
            head_state.pose.orientation.x,
            head_state.pose.orientation.y,
            head_state.pose.orientation.z,
            head_state.pose.orientation.w)
        head_pitch = euler_from_quaternion(quat)[1]
        

        # Calculate errors
        head_x_err = abs(head_pos.x - target_head_position.x)
        head_y_err = abs(head_pos.y - target_head_position.y)
        head_z_err = abs(head_pos.z - target_head_position.z)
        head_p_err = abs(head_pitch - target_head_pitch)

        return (
            twist_x_err, twist_z_err, head_x_err, head_y_err, head_z_err,
            head_p_err)

    
    def calculate_energy_reward(self, joint_state):
        """
        @brief Calculates the energy reward for a particular set of joint
               states. Updates self._energy_reward

        @param joint_state sensor_msgs/JointState The most recently received
                           joint state message

        @returns (void)
        """
        # Which joints do we care about
        measure_energy = [
            "left_wheel_joint", "lower_neck_joint", "lower_tail_joint",
            "right_wheel_joint", "upper_neck_joint", "upper_tail_joint"]

        energies = []
        for i in range(len(joint_state.name)):
            name = joint_state.name[i]
            if name in measure_energy:
                energies.append(abs(
                    joint_state.velocity[i] * joint_state.effort[i]))

        self._energy_reward = 1/(self._energy_mult * sum(energies) + 1)

    
    def joint_state_callback(self, joint_state):
        self.calculate_energy_reward(joint_state)
