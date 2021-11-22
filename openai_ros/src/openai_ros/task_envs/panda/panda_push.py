"""An Openai gym ROS Panda push environment.

Goal:
    In this environment the agent has to learn to push a puck to a desired goal
    position. Based on the `FetchPush-v1 <https://gym.openai.com/envs/FetchPush-v1>`_
    Openai gym environment.
"""

from gym import utils
from openai_ros.task_envs.panda import PandaPickAndPlaceEnv

# Specify topics and other script variables
CONFIG_FILE_PATH = "config/panda_push.yaml"


#################################################
# Panda push environment class ##################
#################################################
class PandaPushEnv(PandaPickAndPlaceEnv, utils.EzPickle):
    """Classed used to create a Panda push environment."""

    def __init__(self, *args, **kwargs):
        """Initializes a Panda Push task environment.

        Args:
            *args: Arguments passed to the
                :obj:`~openai_ros.task_envs.panda.PandaPushEnv` super class.
            **kwargs: Keyword arguments that are passed to the
                :obj:`~openai_ros.task_envs.panda.PandaPushEnv` super class.
        """
        utils.EzPickle.__init__(
            **locals()
        )  # Makes sure the env is pickable when it wraps C++ code.

        super().__init__(
            config_path=CONFIG_FILE_PATH,
            gazebo_world_launch_file="start_push_world.launch",
            *args,
            **kwargs,
        )

    ################################################
    # Overload Reach environment methods ###########
    ################################################
    def _get_params(self):  # noqa: C901
        """Retrieve task environment configuration parameters from parameter server."""
        super()._get_params(ns="panda_push")

    def _init_env_variables(self):
        """Inits variables needed to be initialized each time we reset at the start
        of an episode.
        """
        self._set_init_obj_pose()

        # Sample and visualize goal
        self.goal = self._sample_goal()
        self.goal[-1] = self.object_position[
            -1
        ]  # Make sure object stays on the initial platform
        if self._visualize_target:
            self._visualize_goal()
