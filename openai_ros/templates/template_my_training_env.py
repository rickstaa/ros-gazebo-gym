"""Template file used for creating a new task environments.

This task environment contains a python class that allows to specify the *TASK* that the
robot has to learn. For more information see the
`openai_ros <https://wiki.ros.org/openai_ros>`_ documentation.
"""

import numpy
import rospy
from gym import spaces
from gym.envs.registration import register

# Import your robot environment
from template_my_robot_env import MyRobotEnv

# Register the task environment as a gym environment
timestep_limit_per_episode = 1000
register(
    id="MyTaskEnv-v0",
    entry_point="template_my_task_env:MyTaskEnv",
    timestep_limit=timestep_limit_per_episode,
)


class MyTaskEnv(MyRobotEnv):
    """

    Args:
        MyRobotEnv ([type]): [description]
    """

    def __init__(self):

        # TODO: Implement the action and observation space
        number_actions = rospy.get_param("/my_robot_namespace/n_actions")
        self.action_space = spaces.Discrete(number_actions)

        # TODO: Implement the observation space
        high = numpy.array(
            [
                # Observation 1 max value,
                # Observation 2 max value,
                # ...
                # Observation N max value
            ]
        )
        self.observation_space = spaces.Box(-high, high)

        # TODO: Retrieve required robot variables through the param server, loaded with
        # the ROS launch file.

        # Initiate the robot environment
        super(MyTaskEnv, self).__init__()

    def _set_init_pose(self):
        """Sets the Robot in its init pose.
        """
        # TODO: Implement the logic that sets the robot to it's initial position

    def _init_env_variables(self):
        """Inits variables needed to be initialized each time we reset at the start
        of an episode.
        """
        # TODO: Reset variables that need to be reset at the start of each episode.

    def _set_action(self, action):
        """Move the robot based on the given action.
        """
        # TODO: Implement logic that moves the robot based on a given action

    def _get_obs(self):
        """Here we define what sensor data of our robots observations
        To know which Variables we have acces to, we need to read the
        MyRobotEnv API DOCS
        :return: observations
        """
        # TODO: Implement the logic that retrieves the observation data from the robot
        # sensors.

        # return observations

    def _is_done(self, observations):
        """Checks if episode is done based on the given observations.
        """
        # TODO: Implement logic used to check whether a episode is done.

        # return done

    def _compute_reward(self, observations, done):
        """Computes the reward based on the given observations.
        """
        # TODO: Implement logic that is used to calculate th reward.

        # return reward

    # Internal TaskEnv Methods
