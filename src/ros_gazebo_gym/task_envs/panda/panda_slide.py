"""An ROS Panda slide gymnasium environment.

.. image:: /images/panda/panda_slide_env.png
   :alt: Panda slide environment

This environment is an extension of the :class:`~ros_gazebo_gym.task_envs.panda.panda_pick_and_place.PandaPickAndPlaceEnv` task environment,
sharing most features such as observation and action spaces. Notable distinctions are detailed below.

Goal:
    In this environment the agent has to learn to slide a puck to a desired goal
    position. It was based on the :gymnasium-robotics:`FetchSlide-v2 <envs/fetch/slide/>`
    gymnasium environment.

.. admonition:: Configuration
    :class: important

    The configuration files for this environment are found in the
    :ros-gazebo-gym:`panda task environment config folder <blob/noetic/src/ros_gazebo_gym/task_envs/panda/config/panda_slide.yaml>`.
"""  # noqa: E501

import rospy
from gymnasium import utils
from ros_gazebo_gym.task_envs.panda.markers.puck_marker import PuckMarker
from ros_gazebo_gym.task_envs.panda.panda_pick_and_place import PandaPickAndPlaceEnv

# Specify topics and other script variables.
CONFIG_FILE_PATH = "config/panda_slide.yaml"


#################################################
# Panda slide environment class ##################
#################################################
class PandaSlideEnv(PandaPickAndPlaceEnv, utils.EzPickle):
    """Classed used to create a Panda slide environment."""

    def __init__(self, *args, **kwargs):
        """Initializes a Panda Slide task environment.

        Args:
            *args: Arguments passed to the
                :class:`~ros_gazebo_gym.task_envs.panda.PandaSlideEnv` super class.
            **kwargs: Keyword arguments that are passed to the
                :class:`~ros_gazebo_gym.task_envs.panda.PandaSlideEnv` super class.
        """
        rospy.logwarn("Initialize Panda slide environment.")
        utils.EzPickle.__init__(
            **locals()
        )  # Makes sure the env is pickable when it wraps C++ code.

        super().__init__(
            config_path=CONFIG_FILE_PATH,
            gazebo_world_launch_file="start_slide_world.launch",
            *args,
            **kwargs,
        )

        # Change object marker properties.
        self.object_marker_class = PuckMarker
        self.object_frame_name = "puck"

        rospy.logwarn("Panda slide environment initialized.")

    ################################################
    # Overload Reach environment methods ###########
    ################################################
    def _get_params(self):  # noqa: C901
        """Retrieve task environment configuration parameters from parameter server."""
        super()._get_params(ns="panda_slide")

    def _init_env_variables(self):
        """Inits variables needed to be initialized each time we reset at the start
        of an episode.
        """
        self._set_init_obj_pose()

        # Sample and visualize goal.
        self.goal = self._sample_goal()
        self.goal[-1] = (
            self.object_position[-1] - 0.02
        )  # Make sure object stays in contact with the platform.
        if self._visualize_target:
            self._visualize_goal(
                offset=[0.0, 0.0, 0.02]
            )  # Apply offset to make goal marker visible.
