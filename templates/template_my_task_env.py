"""Template file used for creating a new task environment. It contains a Python class
that allows specifying the **TASK** that the robot has to learn; for more information,
see the `ros_gazebo_gym <https://rickstaa.dev/ros-gazebo-gym>`_ documentation.

Source code
-----------

.. literalinclude:: ../../../../../templates/template_my_task_env.py
   :language: python
   :linenos:
   :lines: 13-
"""

import numpy as np
import rospy
from gymnasium import spaces
from gymnasium.envs.registration import register
from template_my_robot_env import MyRobotEnv  # NOTE: Import your robot environment.

# Register the task environment as a gymnasium environment.
max_episode_steps = 1000
register(
    id="MyTaskEnv-v0",
    entry_point="template_my_task_env:MyTaskEnv",
    max_episode_steps=max_episode_steps,
)


class MyTaskEnv(MyRobotEnv):
    """Environment used to define the task the robot has to learn (i.e. observations
    you want to use, rewards etc).
    """

    def __init__(self):
        """Initializes a new Task environment."""
        # TODO: Implement the action space.
        number_actions = rospy.get_param("/my_robot_namespace/n_actions")
        self.action_space = spaces.Discrete(number_actions)

        # TODO: Implement the observation space.
        high = np.array(
            [
                # Observation 1 max value,
                # Observation 2 max value,
                # ...
                # Observation N max value
            ]
        )
        self.observation_space = spaces.Box(-high, high)

        # TODO: Retrieve required robot variables through the param server.

        # Initiate the Robot environment.
        super(MyTaskEnv, self).__init__()

    ################################################
    # Task environment internal methods ############
    ################################################
    # NOTE: Here you can add additional helper methods that are used in the task env.

    ################################################
    # Overload Robot/Gazebo env virtual methods ####
    ################################################
    # NOTE: Methods that need to be implemented as they are called by the robot and
    # gazebo environments.
    def _set_init_gazebo_variables(self):
        """Initializes variables that need to be initialized at the start of the gazebo
        simulation.
        """
        # TODO: Implement logic that sets initial gazebo physics engine parameters.
        raise NotImplementedError()

    def _set_init_pose(self):
        """Sets the Robot in its initial pose."""
        # TODO: Implement logic that sets the robot to it's initial position.
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
        # TODO: Implement logic that retrieves the observation needed for the reward.
        observations = np.array([0.0, 0.0, 0.0])
        return observations

    def _set_action(self, action):
        """Applies the given action to the simulation.

        Args:
            action (numpy.ndarray): The action we want to apply.
        """
        # TODO: Implement logic that moves the robot based on a given action.
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
