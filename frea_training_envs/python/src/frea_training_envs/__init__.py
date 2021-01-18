from gym.envs.registration import register

register(
    id='FreaMotionControl-v0',
    entry_point='frea_training_envs.task_envs.motion_control:FreaMotionControlEnv',
)
