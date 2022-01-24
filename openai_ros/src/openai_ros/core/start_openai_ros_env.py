#!/usr/bin/env python3
"""Utility function that can be used to start a openai_ros environment.
"""

import gym
import rospy
from openai_ros.core.helpers import register_openai_ros_env


def start_openai_ros_env(task_environment, max_episode_steps=None, **kwargs):
    """Starts a openai_ros gym environment.

    It automates stuff that normally the would have to do:

    1. Registers the TaskEnvironment, if it exists.
    2. Makes the TaskEnvironment.
    3. Checks that the workspace of the user has all that is needed for launching this
       environment. Which means that it will checks if the robot spawn and world spawn
       launch files are there.
    4. Launches the world and robot spawn the robot.

    Args:
        task_environment (str): The name of the openai_ros task environment.
        max_episode_steps (int, optional): The max episode step you want to set for the
            environment. Defaults to ``None`` meaning the value in the config file will
            be used (i.e. :obj:`openai_ros.task_envs.task_envs_list`).
        **kwargs: All kwargs to pass to the gym environment.

    Returns:
        gym.env: The openai_ros task gym environment.

    .. important::
        This method of starting the :module:`openai_ros` task environments has been
        deprecated and will be removed in the future. Please import the
        :module:`openai_ros` package and use the :meth:`gym.make` method instead. See
        `the gym documentation <https://gym.openai.com/docs/>`_ for more information.
    """
    rospy.logwarn(
        "Starting the openai_ros task environments using the `start_openai_ros_env` "
        "method has been deprecated and will be removed in the future. Please import "
        "the `openai_ros` package and use the `gym.make` method instead See the openai "
        "gym documentation https://gym.openai.com/docs/ for more information."
    )
    rospy.loginfo(f"Registering '{task_environment}' openai_ros gym environment...")
    try:
        if (
            task_environment not in gym.envs.registry.env_specs
        ):
            register_openai_ros_env(
                task_env=task_environment, max_episode_steps=max_episode_steps
            )
    except Exception as e:
        rospy.logwarn(e.args[0])

    rospy.loginfo(f"Creating '{task_environment}' openai_ros gym environment...")
    try:
        env = gym.make(task_environment, **kwargs)
    except Exception as e:
        rospy.logwarn(
            "Something went wrong while trying make the "
            f"'{task_environment}' environment."
        )
        raise Exception(
            f"The '{task_environment}' openai_ros gym environment could not be loaded."
        ) from e
    return env
