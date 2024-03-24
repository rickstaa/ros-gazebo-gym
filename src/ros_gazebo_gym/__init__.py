"""Initialize the :ros-gazebo-gym:`ros_gazebo_gym <>` package.

.. important::

    Here all the all :ros-gazebo-gym:`ros_gazebo_gym <>` task environments are
    registered so that they are available in the gymnasium namespace.
"""

from gymnasium.envs.registration import register
from ros_gazebo_gym.task_envs.task_envs_list import ENVS

################################################
# Register task environments ###################
################################################
for env, val in ENVS.items():
    register(
        id=env,
        entry_point=val["entry_point"],
        reward_threshold=val["reward_threshold"],
        max_episode_steps=val["max_steps"],
    )
