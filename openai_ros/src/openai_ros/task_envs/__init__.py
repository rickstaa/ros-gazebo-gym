"""Contains all openai_ros task environments.
"""

import gym
from gym.envs.registration import register
from openai_ros.task_envs.task_envs_list import ENVS

# Load all the openai_ros environments found in the task_env_list
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
