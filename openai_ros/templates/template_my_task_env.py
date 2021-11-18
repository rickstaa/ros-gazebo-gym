"""Template file used for creating a new task environments.

This task environment contains a python class that allows to specify the *TASK* that the
robot has to learn. For more information see the
`openai_ros <https://wiki.ros.org/openai_ros>`_ documentation.
"""

import numpy as np
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
    """Environment used to define the task the robot has to learn (i.e. observations
    you want to use, rewards etc).
    """

    def __init__(self):
        """Initializes a new Task environment."""

        # TODO: Implement the action and observation space
        number_actions = rospy.get_param("/my_robot_namespace/n_actions")
        self.action_space = spaces.Discrete(number_actions)

        # TODO: Implement the observation space
        high = np.array(
            [
                # Observation 1 max value,
                # Observation 2 max value,
                # ...
                # Observation N max value
            ]
        )
        self.observation_space = spaces.Box(-high, high)

        # TODO: Retrieve required robot variables through the param server

        # Initiate the Robot environment
        super(MyTaskEnv, self).__init__()

    ################################################
    # Overload Robot env virtual methods ###########
    ################################################
    # NOTE: Methods that need to be implemented as they are called by the robot and
    # gazebo environments.
    def _set_init_gazebo_variables(self):
        """Initializes variables that need to be initialized at the start of the gazebo
        simulation.
        """
        # TODO: Implement the logic that sets initial gazebo physics engine parameters
        raise NotImplementedError()

    def _set_init_pose(self):
        """Sets the Robot in its initial pose."""
        # TODO: Implement the logic that sets the robot to it's initial position
        return True

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        # TODO: Reset variables that need to be reset at the start of each episode.
        return True

    def _get_obs(self):
        """Here we define what sensor data of our robots observations we have access to.

        Returns:
            numpy.ndarray: The observation data.
        """
        # TODO: Implement the logic that retrieves the observation needed for the reward
        observations = np.array([0.0, 0.0, 0.0])
        return observations

    def _set_action(self, action):
        """Applies the given action to the simulation.

        Args:
            action (numpy.ndarray): The action we want to apply.
        """
        # TODO: Implement logic that moves the robot based on a given action
        return True

    def _is_done(self, observations):
        """Indicates whether or not the episode is done (the robot has fallen for
        example).

        Args:
            observations (numpy.ndarray): The observation vector.

        Returns:
            bool: Whether the episode was finished.
        """
        # TODO: Implement logic used to check whether a episode is done.
        done = True
        return done

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.

        Args:
            observations (numpy.ndarray): The observation vector.
            done (bool): Whether the episode has finished.

        Returns:
            float: The step reward.
        """
        # TODO: Implement logic that is used to calculate the reward.
        reward = 1e6
        return reward

    ################################################
    # Task environment internal methods ############
    ################################################
    # NOTE: Here you can add additional helper methods that are used in the task env
