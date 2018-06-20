from gym import utils
from openai_gazebo import cube_single_disk_env
from gym.envs.registration import register

# Algorithmic
# ----------------------------------------

register(
        id='MovingCubeOneDiskWalk-v0',
        entry_point='openai_gazebo:FetchSlideEnv',
        max_episode_steps=1000,
        reward_threshold=360.0,
    )

class MovingCubeOneDiskWalkEnv(cube_single_disk_env.CubeSingleDiskEnv, utils.EzPickle):
    def __init__(self, reward_type='sparse'):
        initial_qpos = {
            'robot0:slide0': 0.405,
            'robot0:slide1': 0.48,
            'robot0:slide2': 0.0,
            'object0:joint': [1.25, 0.53, 0.4, 1., 0., 0., 0.],
        }
        cube_single_disk_env.CubeSingleDiskEnv.__init__(
            self, 'fetch/push.xml', has_object=True, block_gripper=True, n_substeps=20,
            gripper_extra_height=0.0, target_in_the_air=False, target_offset=0.0,
            obj_range=0.15, target_range=0.15, distance_threshold=0.05,
            initial_qpos=initial_qpos, reward_type=reward_type)
        utils.EzPickle.__init__(self)