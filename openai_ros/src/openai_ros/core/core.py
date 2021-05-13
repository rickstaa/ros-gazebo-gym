#!/usr/bin/env python3
"""Contains several core openai_ros functions. These functions can be used to setup
the openai_ros gym environments for training.
"""

import gym
import rospy
from openai_ros.core.helpers import register_openai_ros_env


def start_openai_ros_env(task_environment):
    """Starts a openai_ros gym environment

    It automates stuff that normally the would have to do:

    1. Registers the TaskEnvironment, if it exists.
    2. Makes the TaskEnvironment.
    3. Checks that the workspace of the user has all that is needed for launching this
       environment. Which means that it will checks if the robot spawn and world spawn
       launch files are there.
    4. Launches the world and robot spawn the robot.

    Args:
        task_environment (str): The name of the openai_ros task environment.

    Returns:
        gym.env: The openai_ros task gym environment.
    """

    rospy.loginfo(f"Registering '{task_environment}' openai_ros gym environment...")
    try:
        register_openai_ros_env(task_env=task_environment, max_episode_steps=10000)
    except Exception as e:
        rospy.logwarn(e.args[0])

    rospy.loginfo(f"Creating '{task_environment}' openai_ros gym environment...")
    try:
        env = gym.make(task_environment)
    except Exception as e:
        rospy.logwarn(
            "Something went wrong while trying make the "
            f"'{task_environment}' environment."
        )
        raise Exception(
            f"The '{task_environment}' openai_ros gym environment could not be loaded."
        ) from e
    return env
