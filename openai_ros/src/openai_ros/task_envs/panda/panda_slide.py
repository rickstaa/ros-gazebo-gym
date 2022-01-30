"""An Openai gym ROS Panda slide environment.

Goal:
    In this environment the agent has to learn to slide a puck to a desired goal
    position. Based on the `FetchSlide-v1 <https://gym.openai.com/envs/FetchSlide-v1>`_
    Openai gym environment.
"""

from gym import utils
from openai_ros.task_envs.panda import PandaPickAndPlaceEnv
from openai_ros.task_envs.panda.markers import PuckMarker

# Specify topics and other script variables
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
                :obj:`~openai_ros.task_envs.panda.PandaSlideEnv` super class.
            **kwargs: Keyword arguments that are passed to the
                :obj:`~openai_ros.task_envs.panda.PandaSlideEnv` super class.
        """
        utils.EzPickle.__init__(
            **locals()
        )  # Makes sure the env is pickable when it wraps C++ code.

        super().__init__(
            config_path=CONFIG_FILE_PATH,
            gazebo_world_launch_file="start_slide_world.launch",
            *args,
            **kwargs,
        )

        # Change object marker properties
        self.object_marker_class = PuckMarker
        self.object_frame_name = "puck"

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

        # Sample and visualize goal
        self.goal = self._sample_goal()
        self.goal[-1] = (
            self.object_position[-1] - 0.02
        )  # Make sure object stays in contact with the platform
        if self._visualize_target:
            self._visualize_goal()
