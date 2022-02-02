"""Contains all :ros_gazebo_gym:`ros_gazebo_gym <>` task environments.

.. autosummary::
   :toctree: _autosummary
   :template: custom-module-template.rst
   :recursive:

   ros_gazebo_gym.task_envs.panda
"""  # NOTE: Autosummary used again here since the code API autosummary didn't pick up the nested envs

import gym
from gym.envs.registration import register
from ros_gazebo_gym.task_envs.task_envs_list import ENVS

# Load all the ros_gazebo_gym environments found in the task_env_list
for env, val in ENVS.items():
    if (
        env not in gym.envs.registry.env_specs
    ):  # NOTE: We don't want to overwrite the environment if it already exists
        register(
            id=env,
            entry_point=val["module"],
            max_episode_steps=val["max_steps"],
            reward_threshold=val["reward_threshold"],
        )
