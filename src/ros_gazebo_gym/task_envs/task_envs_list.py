#!/usr/bin/env python3
"""Contains a list of the available :ros_gazebo_gym:`ros_gazebo_gym <>` gym
environments.

.. important::
    This is where you can register new environments such that they are found inside the
    openai Gym namespace.

Source code
-----------

.. literalinclude:: ../../../../../src/ros_gazebo_gym/task_envs/task_envs_list.py
   :language: python
   :linenos:
   :lines: 18-
"""

# NOTE: Each environment should contain a 'name', 'module' and default 'max_steps' key.
ENVS = {
    # Panda task envs
    "PandaReach-v1": {
        "module": "ros_gazebo_gym.task_envs.panda.panda_reach:PandaReachEnv",
        "max_steps": 100,
        "reward_threshold": 300,
    },
    "PandaPickAndPlace-v1": {
        "module": "ros_gazebo_gym.task_envs.panda.panda_pick_and_place:PandaPickAndPlaceEnv",
        "max_steps": 100,
        "reward_threshold": 300,
    },
    "PandaPush-v1": {
        "module": "ros_gazebo_gym.task_envs.panda.panda_push:PandaPushEnv",
        "max_steps": 100,
        "reward_threshold": 300,
    },
    "PandaSlide-v1": {
        "module": "ros_gazebo_gym.task_envs.panda.panda_slide:PandaSlideEnv",
        "max_steps": 100,
        "reward_threshold": 300,
    },
}  # noqa: E501
