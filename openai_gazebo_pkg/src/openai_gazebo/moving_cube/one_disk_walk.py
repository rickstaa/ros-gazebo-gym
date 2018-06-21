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

    def reward_calculation(self, observations, done):
        #TODO: It has to retrieve the sensor based on observations and done
        raise NotImplementedError()

    def done_calculation(self, observations):
        # TODO: It has to calculate if the episode is done besed on the observations
        raise NotImplementedError()

    def get_robot_state(self):
        # TODO: Get the sensor data and generated the state of the cube
        raise NotImplementedError()
