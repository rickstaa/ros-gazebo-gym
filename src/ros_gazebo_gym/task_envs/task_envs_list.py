"""Contains a list of available :ros-gazebo-gym:`ros_gazebo_gym <>` gymnasium
environments.

.. important::
    This is where you put new environments such that they are registered inside the
    gymnasium namespace.

Source code
-----------

.. literalinclude:: ../../../../../../src/ros_gazebo_gym/task_envs/task_envs_list.py
   :language: python
   :linenos:
   :lines: 17-
"""

# Available environments.
# TODO: Update reward thresholds.
ENVS = {
    # Panda task envs.
    "PandaReach-v1": {
        "entry_point": "ros_gazebo_gym.task_envs.panda.panda_reach:PandaReachEnv",
        "reward_threshold": -20,
        "max_steps": 500,
    },
    "PandaPickAndPlace-v1": {
        "entry_point": "ros_gazebo_gym.task_envs.panda.panda_pick_and_place:PandaPickAndPlaceEnv",  # noqa: E501
        "reward_threshold": -20,
        "max_steps": 1000,
    },
    "PandaPush-v1": {
        "entry_point": "ros_gazebo_gym.task_envs.panda.panda_push:PandaPushEnv",
        "reward_threshold": -20,
        "max_steps": 1000,
    },
    "PandaSlide-v1": {
        "entry_point": "ros_gazebo_gym.task_envs.panda.panda_slide:PandaSlideEnv",
        "reward_threshold": -20,
        "max_steps": 1000,
    },
}
