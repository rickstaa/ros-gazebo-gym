#!/usr/bin/env python3
"""Contains a list of the available :ros_gazebo_gym:`ros_gazebo_gym <>` gym
environments.

NOTE: Here is where you have to PLACE YOUR NEW TASK ENV
"""

# NOTE: Each environment should contain a 'name', 'module' and default 'max_steps' key.
ENVS = {
    # Panda task envs
    "PandaReach-v0": {
        "module": "ros_gazebo_gym.task_envs.panda.panda_reach:PandaReachEnv",
        "max_steps": 100,
        "reward_threshold": 300,
    },
    "PandaPickAndPlace-v0": {
        "module": "ros_gazebo_gym.task_envs.panda.panda_pick_and_place:PandaPickAndPlaceEnv",
        "max_steps": 100,
        "reward_threshold": 300,
    },
    "PandaPush-v0": {
        "module": "ros_gazebo_gym.task_envs.panda.panda_push:PandaPushEnv",
        "max_steps": 100,
        "reward_threshold": 300,
    },
    "PandaSlide-v0": {
        "module": "ros_gazebo_gym.task_envs.panda.panda_slide:PandaSlideEnv",
        "max_steps": 100,
        "reward_threshold": 300,
    },
}  # noqa: E501
