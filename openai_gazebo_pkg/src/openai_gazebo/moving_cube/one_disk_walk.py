from gym import utils
from openai_gazebo import cube_single_disk_env
from gym.envs.registration import register

# Algorithmic
# ----------------------------------------

# The path is __init__.py of openai_gazebo, where we import the MovingCubeOneDiskWalkEnv directly
register(
        id='MovingCubeOneDiskWalk-v0',
        entry_point='openai_gazebo:MovingCubeOneDiskWalkEnv',
        timestep_limit=1000,
    )

class MovingCubeOneDiskWalkEnv(cube_single_disk_env.CubeSingleDiskEnv):
    def __init__(self):
        # TODO: Get this number of actions from elsewhere
        n_actions = 3

        # Here we will add any init functions prior to starting the CubeSingleDiskEnv
        super(MovingCubeOneDiskWalkEnv, self).__init__(
            self, test_cubesinglediskenc_arg="TestValue", n_actions=n_actions)