"""An Openai gym ROS Panda reach environment.

.. figure:: /images/panda/panda_reach_env.png
   :alt: Panda reach environment

Goal:
    In this environment the agent has to learn to move the panda robot to a given goal
    position. Based on the `FetchReach-v1 <https://gym.openai.com/envs/FetchReach-v1>`_
    Openai gym environment.

.. admonition:: Configuration
    :class: important

    The configuration files for this environment are found in the
    `panda task environment config folder <../config/panda_reach.yaml>`_).

"""

import os
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import rospy
import tf2_geometry_msgs
from franka_msgs.srv import ResetJointStates, ResetJointStatesRequest
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3
from gym import spaces, utils
from ros_gazebo_gym.common import Singleton
from ros_gazebo_gym.common.functions import (
    flatten_list,
    gripper_width_2_finger_joints_positions,
    list_2_human_text,
    lower_first_char,
    merge_n_dicts,
    normalize_quaternion,
    pose_msg_2_pose_dict,
    split_bounds_dict,
    split_pose_dict,
)
from ros_gazebo_gym.common.markers import SampleRegionMarker, TargetMarker
from ros_gazebo_gym.core import ROSLauncher
from ros_gazebo_gym.core.helpers import get_log_path, load_ros_params_from_yaml
from ros_gazebo_gym.exceptions import EePoseLookupError
from ros_gazebo_gym.robot_envs.panda_env import PandaEnv
from rospy.exceptions import ROSException, ROSInterruptException
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

try:
    import panda_gazebo.msg as pg_msg
    import panda_gazebo.srv as pg_srv

    PANDA_GAZEBO_IMPORTED = True
except ImportError:
    PANDA_GAZEBO_IMPORTED = False


# Specify topics and other script variables
CONNECTION_TIMEOUT = 5  # Timeout for connecting to services or topics
MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC = (
    "panda_moveit_planner_server/get_random_joint_positions"
)
MOVEIT_SET_JOINT_POSITIONS_TOPIC = "panda_moveit_planner_server/set_joint_positions"
MOVEIT_GET_RANDOM_EE_POSE_TOPIC = "panda_moveit_planner_server/get_random_ee_pose"
MOVEIT_ADD_PLANE_TOPIC = "panda_moveit_planner_server/planning_scene/add_plane"
FRANKA_GAZEBO_RESET_JOINT_STATES_TOPIC = "reset_joint_states"
VALID_EE_CONTROL_JOINTS = ["x", "y", "z", "rx", "ry", "rz", "rw"]
CONFIG_FILE_PATH = "config/panda_reach.yaml"
PANDA_REST_CONFIGURATION = [
    0,
    0,
    0,
    -1.57079632679,
    0,
    1.57079632679,
    0.785398163397,
    0.001,
    0.001,
]
LOG_STEP_DEBUG_INFO = False


#################################################
# Panda reach environment class #################
#################################################
class PandaReachEnv(PandaEnv, utils.EzPickle, metaclass=Singleton):
    """Class that provides all the methods used for the algorithm training.

    Attributes:
        action_space (:obj:`gym.spaces.box.Box`): Gym action space object.
        observation_space (:obj:`gym.spaces.dict.Dict`): Gym observation space object.
        goal (:obj:`geometry_msgs.PoseStamped`): The current goal.
    """

    def __init__(  # noqa: C901
        self,
        control_type="effort",
        config_path=CONFIG_FILE_PATH,
        gazebo_world_launch_file="start_reach_world.launch",
        visualize=None,
    ):
        """Initializes a Panda Task Environment.

        Args:
            control_Type (str, optional): The type of control you want to use for the
                panda robot (i.e. hand and arm). Options are: ``trajectory``,
                ``position``, ``effort`` or ``end_effector``. Defaults to ``effort``.
            config_path (str, optional): Path where the environment configuration
                value are found. The path is resolved relative to the
                :class:`~ros_gazebo_gym.task_envs.panda.panda_reach` class file.
            gazebo_world_launch_file (str, optional): Name of the launch file that loads
                the gazebo world. Currently only the launch files inside the
                `panda_gazebo <https://github.com/rickstaa/panda-gazebo>`_ package are
                supported. Defaults to ``start_reach_world.launch``.
            visualize (bool, optional): Whether you want to show the RVIZ visualization.
                Defaults to ``None`` meaning the task configuration file values will
                be used.

        .. important::
            In this environment, the joint trajectory control is not implemented yet for
            multiple waypoints. This is because the action space only contains one
            waypoint. The :obj:`~PandaEnv.set_arm_joint_trajectory` method, however,
            already accepts multiple waypoints. As a result, task environment can be
            easily extended to work with multiple waypoints by modifying the
            :obj:`PandaReachEnv~._create_action_space` method.
        """
        utils.EzPickle.__init__(
            **locals()
        )  # Makes sure the env is pickable when it wraps C++ code.
        ROSLauncher.initialize()  # Makes sure roscore is running and ROS is initialized

        # This is the path where the simulation files will be downloaded if not present
        workspace_path = rospy.get_param("/panda_reach_v0/workspace_path", None)
        if workspace_path:
            assert os.path.exists(workspace_path), (
                "The Simulation ROS Workspace path "
                + workspace_path
                + " DOESN'T exist, execute: mkdir -p "
                + workspace_path
                + "/src;cd "
                + workspace_path
                + ";catkin_make"
            )

        # Load Params from the desired Yaml file relative to this TaskEnvironment
        rospy.logdebug("Load Panda Reach parameters.")
        self._config_file_path = Path(__file__).parent.joinpath(config_path)
        load_ros_params_from_yaml(
            package_name="ros_gazebo_gym",
            rel_path_from_package_to_file=self._config_file_path.parent,
            yaml_file_name=self._config_file_path.name,
        )
        self._get_params()

        # Thrown warning if gazebo is already running
        if any(
            ["/gazebo" in topic for topic in flatten_list(rospy.get_published_topics())]
        ):
            rospy.logerr(
                f"Shutting down '{rospy.get_name()}' since a Gazebo instance is "
                "already running. Unfortunately, spawning multiple Panda simulations "
                "is not yet supported. Please shut down this instance and try again."
            )
            sys.exit(0)

        # Launch the panda task gazebo environment (Doesn't yet add the robot)
        # NOTE: This downloads and builds the required ROS packages if not found
        launch_log_file = (
            str(
                get_log_path()
                .joinpath(
                    "{}_{}.log".format(
                        gazebo_world_launch_file.replace(".", "_"),
                        datetime.now().strftime("%d_%m_%Y_%H_%M_%S"),
                    )
                )
                .resolve()
            )
            if not self._roslaunch_log_to_console
            else None
        )
        ROSLauncher.launch(
            package_name="panda_gazebo",
            launch_file_name=gazebo_world_launch_file,
            workspace_path=workspace_path,
            gazebo_gui=self._gazebo_gui,
            pause=True,
            physics=self._physics,
            log_file=launch_log_file,
        )

        # Throw error if the panda_gazebo package was downloaded in the same run
        # NOTE: Needed because of how ROS adds custom ROS python modules to the
        # python path. It uses the '__path__' variable to do this. This method however
        # only works before the code file is executed (see
        # https://docs.python.org/3/tutorial/modules.html for more information).
        # I did not find a easy fix to solve this behaviors.
        if not PANDA_GAZEBO_IMPORTED:
            rospy.logerr(
                "The 'panda_gazebo' package was not found in your PYTHONPATH. If the "
                "'panda_gazebo' package was just downloaded, it is not yet available "
                "due to how ROS adds custom python modules to the PYTHONPATH. As a "
                "result, you have to restart the training script before using the "
                "Panda ros_gazebo_gym environment."
            )
            sys.exit(0)

        ########################################
        # Initiate Robot environments ##########
        ########################################

        # Initialize the Robot environment
        super(PandaReachEnv, self).__init__(
            robot_EE_link=self._ee_link,
            load_gripper=self._load_gripper,
            block_gripper=self._block_gripper,
            control_type=control_type,
            workspace_path=workspace_path,
            log_reset=self._log_reset,
            visualize=visualize,
        )

        # Initialize task environment objects
        self._is_done_samples = 0
        self._init_model_configuration = {}

        ########################################
        # Connect to required services, ########
        # subscribers and publishers. ##########
        ########################################

        # Connect to MoveIt 'get_random_joint_positions' service
        try:
            moveit_get_random_joint_positions_srv_topic = (
                f"{self.robot_name_space}/{MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC}"
            )
            rospy.logdebug(
                "Connecting to '%s' service."
                % moveit_get_random_joint_positions_srv_topic
            )
            rospy.wait_for_service(
                moveit_get_random_joint_positions_srv_topic,
                timeout=CONNECTION_TIMEOUT,
            )
            self._moveit_get_random_joint_positions_client = rospy.ServiceProxy(
                moveit_get_random_joint_positions_srv_topic,
                pg_srv.GetRandomJointPositions,
            )
            rospy.logdebug(
                "Connected to '%s' service!"
                % moveit_get_random_joint_positions_srv_topic
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!"
                % moveit_get_random_joint_positions_srv_topic
            )

        # Connect to MoveIt 'get_random_ee_pose' service
        try:
            moveit_get_random_ee_pose_srv_topic = (
                f"{self.robot_name_space}/{MOVEIT_GET_RANDOM_EE_POSE_TOPIC}"
            )
            rospy.logdebug(
                "Connecting to '%s' service." % moveit_get_random_ee_pose_srv_topic
            )
            rospy.wait_for_service(
                moveit_get_random_ee_pose_srv_topic,
                timeout=CONNECTION_TIMEOUT,
            )
            self._moveit_get_random_ee_pose_client = rospy.ServiceProxy(
                moveit_get_random_ee_pose_srv_topic, pg_srv.GetRandomEePose
            )
            rospy.logdebug(
                "Connected to '%s' service!" % moveit_get_random_ee_pose_srv_topic
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!"
                % moveit_get_random_ee_pose_srv_topic
            )

        # Connect to MoveIt 'planning_scene/add_plane' service
        try:
            moveit_add_plane_srv_topic = (
                f"{self.robot_name_space}/{MOVEIT_ADD_PLANE_TOPIC}"
            )
            rospy.logdebug("Connecting to '%s' service." % moveit_add_plane_srv_topic)
            rospy.wait_for_service(
                moveit_add_plane_srv_topic,
                timeout=CONNECTION_TIMEOUT,
            )
            self._moveit_add_plane_srv = rospy.ServiceProxy(
                moveit_add_plane_srv_topic, pg_srv.AddPlane
            )
            rospy.logdebug("Connected to '%s' service!" % moveit_add_plane_srv_topic)
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % moveit_add_plane_srv_topic
            )

        if self._moveit_init_pose_control:
            # Connect to MoveIt 'planning_scene/set_joint_positions' service
            try:
                moveit_set_joint_positions_srv_topic = (
                    f"{self.robot_name_space}/{MOVEIT_SET_JOINT_POSITIONS_TOPIC}"
                )
                rospy.logdebug(
                    "Connecting to '%s' service." % moveit_set_joint_positions_srv_topic
                )
                rospy.wait_for_service(
                    moveit_set_joint_positions_srv_topic,
                    timeout=CONNECTION_TIMEOUT,
                )
                self._moveit_set_joint_positions_srv = rospy.ServiceProxy(
                    moveit_set_joint_positions_srv_topic, pg_srv.SetJointPositions
                )
                rospy.logdebug(
                    "Connected to '%s' service!" % moveit_set_joint_positions_srv_topic
                )
            except (rospy.ServiceException, ROSException, ROSInterruptException):
                rospy.logwarn(
                    "Failed to connect to '%s' service!"
                    % moveit_set_joint_positions_srv_topic
                )
        else:
            # Connect to franka_gazebo's 'reset_joint_positions' service
            try:
                franka_gazebo_reset_joint_states_srv_topic = (
                    f"{self.robot_name_space}/{FRANKA_GAZEBO_RESET_JOINT_STATES_TOPIC}"
                )
                rospy.logdebug(
                    "Connecting to '%s' service."
                    % franka_gazebo_reset_joint_states_srv_topic
                )
                rospy.wait_for_service(
                    franka_gazebo_reset_joint_states_srv_topic,
                    timeout=CONNECTION_TIMEOUT,
                )
                self._franka_gazebo_reset_joint_states_srv = rospy.ServiceProxy(
                    franka_gazebo_reset_joint_states_srv_topic, ResetJointStates
                )
                rospy.logdebug(
                    "Connected to '%s' service!"
                    % franka_gazebo_reset_joint_states_srv_topic
                )
            except (rospy.ServiceException, ROSException, ROSInterruptException):
                rospy.logwarn(
                    "Failed to connect to '%s' service!"
                    % franka_gazebo_reset_joint_states_srv_topic
                )

        # Create current target publisher
        rospy.logdebug("Creating target pose publisher.")
        self._target_pose_marker_pub = rospy.Publisher(
            "/ros_gazebo_gym/current_target", TargetMarker, queue_size=1, latch=True
        )
        rospy.logdebug("Goal target publisher created.")

        if self._load_rviz:
            # Create target bounding region publisher
            rospy.logdebug("Creating target bounding region publisher.")
            self._target_sample_region_marker_pub = rospy.Publisher(
                "/ros_gazebo_gym/target_sample_region",
                SampleRegionMarker,
                queue_size=1,
                latch=True,
            )
            rospy.logdebug("Target bounding region publisher created.")

            # Create initial pose bounding region publisher
            rospy.logdebug("Creating initial pose sample region publisher.")
            self._init_pose_sample_region_marker__pub = rospy.Publisher(
                "/ros_gazebo_gym/init_pose_sample_region",
                SampleRegionMarker,
                queue_size=1,
                latch=True,
            )
            rospy.logdebug("Initial pose sample region publisher created.")

        ########################################
        # Initialize rviz visualizations #######
        ########################################

        # Add ground to MoveIt planning scene
        self._add_ground_to_moveit_scene()

        # Add pose and target sampling bounds to rviz
        self._init_rviz_visualizations()

        ########################################
        # Create action and observation space ##
        ########################################
        rospy.logdebug("Setup gym action and observation space.")

        self.action_space = self._create_action_space()
        self.goal = self._sample_goal()
        obs = self._get_obs()
        self.observation_space = spaces.Dict(
            dict(
                desired_goal=spaces.Box(
                    -np.inf, np.inf, shape=obs["desired_goal"].shape, dtype="float32"
                ),
                achieved_goal=spaces.Box(
                    -np.inf, np.inf, shape=obs["achieved_goal"].shape, dtype="float32"
                ),
                observation=spaces.Box(
                    -np.inf, np.inf, shape=obs["observation"].shape, dtype="float32"
                ),
            )
        )

    ################################################
    # Task environment internal methods ############
    ################################################
    # NOTE: Here you can add additional helper methods that are used in the task env

    def _init_rviz_visualizations(self):
        """Add pose and target sampling bounds to rviz."""
        # Add goal samping bounds to rviz
        if (
            self._target_sampling_strategy != "fixed"
            and self._load_rviz
            and self._visualize_target_sampling_bounds
        ):
            goal_sample_region_marker_msg = SampleRegionMarker(
                x_min=self._target_sampling_bounds["x_min"],
                y_min=self._target_sampling_bounds["y_min"],
                z_min=self._target_sampling_bounds["z_min"],
                x_max=self._target_sampling_bounds["x_max"],
                y_max=self._target_sampling_bounds["y_max"],
                z_max=self._target_sampling_bounds["z_max"],
                id=0,
            )
            self._target_sample_region_marker_pub.publish(goal_sample_region_marker_msg)

        # Add initial pose sampling region to rviz
        if (
            self._load_rviz
            and self._visualize_init_pose_bounds
            and self._init_pose_sampling_bounds is not None
            and self._pose_sampling_type != "joint_positions"
        ):
            init_pose_sample_region_marker_msg = SampleRegionMarker(
                x_min=self._init_pose_sampling_bounds["x_min"],
                y_min=self._init_pose_sampling_bounds["y_min"],
                z_min=self._init_pose_sampling_bounds["z_min"],
                x_max=self._init_pose_sampling_bounds["x_max"],
                y_max=self._init_pose_sampling_bounds["y_max"],
                z_max=self._init_pose_sampling_bounds["z_max"],
                id=1,
            )
            init_pose_region_color = ColorRGBA()
            init_pose_region_color.a = 0.15
            init_pose_region_color.r = 0.0
            init_pose_region_color.g = 0.0
            init_pose_region_color.b = 1.0
            init_pose_sample_region_marker_msg.color = init_pose_region_color
            self._init_pose_sample_region_marker__pub.publish(
                init_pose_sample_region_marker_msg
            )

    def _add_ground_to_moveit_scene(self):
        """Adds the ground to the MoveIt Planning scence."""
        if hasattr(self, "_moveit_add_plane_srv"):
            add_ground_req = pg_srv.AddPlaneRequest(
                name="ground",
                pose=Pose(orientation=Quaternion(0, 0, 0, 1)),
                normal=[0, 0, 1],
            )
            resp = self._moveit_add_plane_srv.call(add_ground_req)
            if not resp.success:
                rospy.logerr(
                    "The ground was not added successfully to the MoveIt planning "
                    "scene. As a result, MoveIt is unaware of the ground's existence "
                    "and may plan states that are in collision."
                )
        else:
            rospy.logerr(
                f"Failled to connect to '{MOVEIT_ADD_PLANE_TOPIC}' service! "
                f"Shutting down '{rospy.get_name()}' as this service is required for "
                "making MoveIt aware of the ground."
            )
            sys.exit(0)

    def _get_params(self, ns="panda_reach"):  # noqa: C901
        """Retrieve task environment configuration parameters from parameter server.

        Args:
            ns (str, optional): The namespace on which the parameters are found.
                Defaults to "panda_reach".
        """
        try:
            # Retrieve control variables
            try:
                self._direct_control = rospy.get_param(f"/{ns}/control/direct_control")
            except KeyError:
                self._direct_control = True
            try:
                self._ee_link = rospy.get_param(f"/{ns}/control/ee_link")
            except KeyError:
                self._ee_link = "panda_link8"
            try:
                self._load_gripper = rospy.get_param(f"/{ns}/control/load_gripper")
            except KeyError:
                self._load_gripper = True
            try:
                self._block_gripper = rospy.get_param(f"/{ns}/control/block_gripper")
            except KeyError:
                self._block_gripper = False
            try:
                self._grasping = rospy.get_param(f"/{ns}/control/grasping")
            except KeyError:
                self._grasping = None
            try:
                self._arm_wait = rospy.get_param(f"/{ns}/control/arm_wait")
            except KeyError:
                self._arm_wait = False
            try:
                self._hand_wait = rospy.get_param(f"/{ns}/control/hand_wait")
            except KeyError:
                self._hand_wait = True
            try:
                self._controlled_joints = rospy.get_param(
                    f"/{ns}/control/controlled_joints"
                )
            except KeyError:
                self._controlled_joints = None
            try:
                self._ee_control_coordinates = rospy.get_param(
                    f"/{ns}/control/ee_control_coordinates"
                )
            except KeyError:
                self._ee_control_coordinates = None
            # Retrieve sampling variables
            try:
                self._visualize_init_pose_bounds = rospy.get_param(
                    f"/{ns}/pose_sampling/visualize_init_pose_bounds"
                )
            except KeyError:
                self._visualize_init_pose_bounds = True
            try:
                self._reset_init_pose = rospy.get_param(
                    f"/{ns}/pose_sampling/reset_init_pose"
                )
            except KeyError:
                self._reset_init_pose = True
            try:
                self._random_init_pose = rospy.get_param(
                    f"/{ns}/pose_sampling/random_init_pose"
                )
            except KeyError:
                self._random_init_pose = True
            try:
                self._randomize_first_episode = rospy.get_param(
                    f"/{ns}/pose_sampling/randomize_first_episode"
                )
            except KeyError:
                self._randomize_first_episode = True
            try:
                self._pose_sampling_attempts = rospy.get_param(
                    f"/{ns}/pose_sampling/attempts"
                )
            except KeyError:
                self._pose_sampling_attempts = 10
            try:
                self._pose_sampling_type = rospy.get_param(
                    f"/{ns}/pose_sampling/pose_sampling_type"
                ).lower()
            except KeyError:
                self._pose_sampling_type = "end_effector_pose"
            try:
                self._moveit_init_pose_control = rospy.get_param(
                    f"/{ns}/pose_sampling/moveit_control"
                )
            except KeyError:
                self._moveit_init_pose_control = False
            try:
                self._init_pose = rospy.get_param(f"/{ns}/pose_sampling/init_pose")
            except KeyError:
                self._init_pose = {
                    "x": 0.23,
                    "y": 0.29,
                    "z": 0.35,
                    "rx": 0.78,
                    "ry": 0.62,
                    "rz": -0.0,
                    "rw": 4.42,
                    "panda_joint1": 0.0,
                    "panda_joint2": 0.0,
                    "panda_joint3": 0.0,
                    "panda_joint4": -1.57079632679,
                    "panda_joint5": 0.0,
                    "panda_joint6": 1.57079632679,
                    "panda_joint7": 0.785398163397,
                    "gripper_width": 0.001,
                }
            try:
                self._init_pose_offset = rospy.get_param(f"/{ns}/pose_sampling/offset")
            except KeyError:
                self._init_pose_offset = {"x": 0.0, "y": 0.0, "z": 0.0}
            try:
                self._init_pose_sampling_bounds = rospy.get_param(
                    f"/{ns}/pose_sampling/bounds"
                )
            except KeyError:
                self._init_pose_sampling_bounds = None
            try:
                self._visualize_target = rospy.get_param(
                    f"/{ns}/target_sampling/visualize_target"
                )
            except KeyError:
                self._visualize_target = True
            try:
                self._visualize_target_sampling_bounds = rospy.get_param(
                    f"/{ns}/target_sampling/visualize_target_bounds"
                )
            except KeyError:
                self._visualize_target_sampling_bounds = True
            try:
                self._target_sampling_strategy = rospy.get_param(
                    f"/{ns}/target_sampling/strategy"
                )
            except KeyError:
                self._target_sampling_strategy = "global"
            if self._target_sampling_strategy != "fixed":
                try:
                    self._target_sampling_bounds = rospy.get_param(
                        f"/{ns}/target_sampling/bounds/"
                        f"{self._target_sampling_strategy}"
                    )
                except KeyError:
                    self._target_sampling_bounds = {
                        "x_min": -0.7,
                        "x_max": 0.7,
                        "y_min": -0.7,
                        "y_max": 0.7,
                        "z_min": 0.0,
                        "z_max": 1.3,
                    }
            try:
                self._fixed_target_pose = rospy.get_param(
                    f"/{ns}/target_sampling/fixed_target"
                )
            except KeyError:
                self._fixed_target_pose = {
                    "x": 0.4,
                    "y": 0.0,
                    "z": 0.8,
                }
            try:
                self._target_offset = rospy.get_param(f"/{ns}/target_sampling/offset")
            except KeyError:
                self._target_ofset = {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                }
            # Retrieve reward variables
            try:
                self._reward_type = rospy.get_param(f"/{ns}/training/reward_type")
            except KeyError:
                self._reward_type = "sparse"
            try:
                self._target_hold = rospy.get_param(f"/{ns}/training/target_hold")
            except KeyError:
                self._target_hold = True
            try:
                self._hold_samples = rospy.get_param(f"/{ns}/training/hold_samples")
            except KeyError:
                self._hold_samples = 2
            try:
                self._distance_threshold = rospy.get_param(
                    f"/{ns}/training/distance_threshold"
                )
            except KeyError:
                self._distance_threshold = 0.05
            try:
                self._collision_penalty = rospy.get_param(
                    f"/{ns}/training/collision_penalty"
                )
            except KeyError:
                self._collision_penalty = 0.0
            try:
                self._ee_frame_offset = rospy.get_param(
                    f"/{ns}/training/ee_frame_offset"
                )
            except KeyError:
                self._ee_frame_offset = None
            # Retrieve environment variables
            try:
                self._action_bounds = rospy.get_param(
                    f"/{ns}/environment/action_space/bounds"
                )
            except KeyError:
                self._action_bounds = {
                    "low": {
                        "x": -1.3,
                        "y": -1.3,
                        "z": 0.0,
                        "rx": 0,
                        "ry": 0,
                        "rz": 0,
                        "rw": 0,
                    },
                    "high": {
                        "x": 1.3,
                        "y": 1.3,
                        "z": 1.3,
                        "rx": 1,
                        "ry": 1,
                        "rz": 1,
                        "rw": 1,
                    },
                }
            # Retrieve global variables
            try:
                self._load_rviz = rospy.get_param(f"/{ns}/load_rviz")
            except KeyError:
                self._load_rviz = True
            try:
                self._rviz_file = Path(__file__).parent.joinpath(
                    rospy.get_param(f"/{ns}/rviz_file")
                )
            except KeyError:
                self._rviz_file = Path(__file__).parent.joinpath("config/moveit.rviz")
            try:
                self._physics = rospy.get_param(f"/{ns}/physics").lower()
            except KeyError:
                self._physics = "dart"
            try:
                self._gazebo_gui = rospy.get_param(f"/{ns}/load_gazebo_gui")
            except KeyError:
                self._gazebo_gui = True
            try:
                self._log_reset = rospy.get_param(f"/{ns}/log_reset")
            except KeyError:
                self._log_reset = False
            try:
                self._log_step_debug_info = rospy.get_param(
                    f"/{ns}/log_step_debug_info"
                )
            except KeyError:
                self._log_step_debug_info = False
            try:
                self._roslaunch_log_to_console = rospy.get_param(
                    f"/{ns}/roslaunch_log_to_console"
                )
            except KeyError:
                self._roslaunch_log_to_console = False
        except KeyError as e:
            rospy.logerr(
                f"Parameter '{e.args[0]}' could not be retrieved from the parameter "
                "server. Please make sure this parameter is present in the Panda task "
                f"environment configuration file '{self._config_file_path}' and try "
                "again."
            )
            sys.exit(0)

    def _robot_get_obs(self):
        """Returns all joint positions and velocities associated with a robot.

        Returns:
            :obj:`numpy.array`: Robot Positions, Robot Velocities
        """
        data = self.joint_states
        if data.position is not None and data.name:
            return (
                np.array(data.position),
                np.array(data.velocity),
            )
        else:
            return np.zeros(0), np.zeros(0)

    def _goal_distance(self, goal_a, goal_b):
        """Calculates the perpendicular distance to the goal.

        Args:
            goal_a (:obj:`numpy.ndarray`): List containing a gripper and object pose.
            goal_b (:obj:`numpy.ndarray`): List containing a gripper and object pose.

        Returns:
            :obj:`numpy.float32`: Perpendicular distance to the goal.
        """
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)

    def _get_random_joint_positions(self):
        """Get valid joint position commands for the Panda arm and hand.

        Returns:
            dict: Dictionary containing a valid joint position for each joint. Returns
                a empty dictionary if no valid joint positions were found.
        """
        if hasattr(self, "_moveit_get_random_joint_positions_client"):
            req = pg_srv.GetRandomJointPositionsRequest()
            req.attempts = self._pose_sampling_attempts

            # Apply pose bounding region
            if (
                hasattr(self, "_init_pose_sampling_bounds")
                and self._init_pose_sampling_bounds is not None
            ):
                _, joint_positions_bound_region = split_bounds_dict(
                    self._init_pose_sampling_bounds
                )
                joint_positions_bound_region = gripper_width_2_finger_joints_positions(
                    joint_positions_bound_region, self.joints["hand"]
                )
                joint_limits = pg_msg.JointLimits()
                joint_limits.names = list(joint_positions_bound_region.keys())
                joint_limits.values = list(joint_positions_bound_region.values())
                req.joint_limits = joint_limits

            # Retrieve random joint pose and return
            resp = self._moveit_get_random_joint_positions_client.call(req)
            if resp.success:
                joint_positions_dict = dict(zip(resp.joint_names, resp.joint_positions))
                return joint_positions_dict
            else:
                return {}
        else:
            rospy.logerr(
                f"Failled to connect to '{MOVEIT_GET_RANDOM_JOINT_POSITIONS_TOPIC}' "
                f"service! Shutting down '{rospy.get_name()}' as this service is "
                "required for retrieving random joint positions."
            )
            sys.exit(0)

    def _get_random_ee_pose(self):
        """Get a valid random EE pose that considers the boundaries set in the task
        environment configuration file.

        Returns:
            (tuple): tuple containing:

                - random_ee_pose (dict): Random EE pose. A empty dictionary is returned
                    when no valid random EE pose could be found.
                - model_configuration (dict): A set of joint positions that result in
                    this EE pose. A empty dictionary is returned when no valid random
                    EE pose could be found.
        """
        if hasattr(self, "_moveit_get_random_ee_pose_client"):
            req = pg_srv.GetRandomEePoseRequest()
            req.attempts = self._pose_sampling_attempts

            # Apply pose bounding region
            if (
                hasattr(self, "_init_pose_sampling_bounds")
                and self._init_pose_sampling_bounds is not None
            ):
                ee_pose_bound_region, _ = split_bounds_dict(
                    self._init_pose_sampling_bounds
                )
                req.bounding_region = pg_msg.BoundingRegion(**ee_pose_bound_region)

            # Retrieve random ee pose and return result
            resp = self._moveit_get_random_ee_pose_client.call(req)
            if resp.success:
                return pose_msg_2_pose_dict(resp.ee_pose), dict(
                    zip(resp.joint_names, resp.joint_positions)
                )
            else:
                return {}, {}
        else:
            rospy.logerr(
                f"Failled to connect to '{MOVEIT_GET_RANDOM_EE_POSE_TOPIC}' service! "
                f"Shutting down '{rospy.get_name()}' as this service is required for "
                "retrieving a random end effector pose."
            )
            sys.exit(0)

    def _clip_goal_position(self, goal_pose):
        """Limit the possible goal position x, y and z values to a certian range.

        Args:
            goal_pose (:obj:`numpy.ndarray`): A numpy array containing the goal x,y and
                z values.
        """
        min_bounds = [
            val for key, val in self._target_sampling_bounds.items() if "min" in key
        ]
        max_bounds = [
            val for key, val in self._target_sampling_bounds.items() if "max" in key
        ]
        clipped_goal_pose = np.clip(goal_pose, a_min=min_bounds, a_max=max_bounds)
        if any(clipped_goal_pose != goal_pose):
            rospy.logwarn(
                "Goal pose clipped since it was not within the set target bounds."
            )
            rospy.logdebug("New goal pose: %s" % clipped_goal_pose)
        return clipped_goal_pose

    def _check_config_action_space_joints(self):  # noqa: C901
        """Validates whether the 'ee_control_coordinates' and 'controlled_joints' fields
        in the task environment configuration file contain vallid joints/coordinates
        that can be used as action space joints.

        Raises:
            SystemExit: Thrown when the coordinates/joints in the task environment
                config file are invalid. As a result the ROS script is shutdown.
        """
        # Validate EE coordinates
        invalid_ee_coordinates = []
        if self.robot_control_type == "end_effector":
            if self._ee_control_coordinates:
                for joint in self._ee_control_coordinates:
                    if joint not in VALID_EE_CONTROL_JOINTS:
                        invalid_ee_coordinates.append(joint)

        # Validate control_joints
        hand_joints = ["gripper_width", "gripper_max_effort"]
        invalid_joints = []
        if self._controlled_joints:
            valid_joints = self.joints["arm"] + hand_joints
            for joint in self._controlled_joints:
                if joint not in valid_joints:
                    invalid_joints.append(joint)
        if not self._load_gripper:
            self._controlled_joints = [
                item for item in self._controlled_joints if item not in hand_joints
            ]

        # Throw error and shutdown node if invalid joint was found
        if invalid_ee_coordinates or invalid_joints:
            error_msg = "Shutting down '%s' since the " % rospy.get_name()
            if invalid_ee_coordinates and invalid_joints:
                invalid = flatten_list([invalid_ee_coordinates, invalid_joints])
                error_msg += list_2_human_text(
                    ["'" + item + "'" for item in invalid],
                    end_seperator="and",
                )
                error_msg += (
                    " %s that %s specified in the 'ee_control_coordinates' and "
                    "'controlled_joints' task environment config variables were "
                    "invalid."
                ) % (
                    ("coordinate/joint", "was")
                    if len(invalid) == 1
                    else ("coordinates/joints", "were")
                )
            elif invalid_ee_coordinates:
                error_msg += list_2_human_text(
                    ["'" + item + "'" for item in invalid_ee_coordinates],
                    end_seperator="and",
                )
                error_msg += (
                    " %s that %s specified in the 'ee_control_coordinates' task "
                    "environment configuration variable %s invalid."
                ) % (
                    ("coordinate", "was", "is")
                    if len(invalid_ee_coordinates) == 1
                    else ("coordinates", "were", "are")
                )
            else:
                error_msg += list_2_human_text(
                    ["'" + item + "'" for item in invalid_joints],
                    end_seperator="and",
                )
                error_msg += (
                    " %s that %s specified in the 'controlled_joints' task "
                    "environment configuration variable %s invalid."
                ) % (
                    ("joint", "was", "is")
                    if len(invalid_joints) == 1
                    else ("joints", "were", "are")
                )
            rospy.logerr(error_msg)
            sys.exit(0)

    def _get_action_space_joints(self):
        """Retrieves the joints that are being controlled when we sample from the action
        space.

        Returns:
            list: Joints that are controlled.
        """
        # Validate the action space joints set in the config file
        self._check_config_action_space_joints()

        # Get action space joints
        if self.robot_control_type == "end_effector":
            # Get end effector control coordinates
            if not self._ee_control_coordinates:
                action_space_joints = ["x", "y", "z", "rx", "ry", "rz", "rw"]
            else:
                action_space_joints = self._ee_control_coordinates

            # Get gripper control joints
            if self._load_gripper:
                action_space_joints.extend(["gripper_width", "gripper_max_effort"])
        else:
            if self._controlled_joints:
                action_space_joints = self._controlled_joints
            else:
                action_space_joints = self.joints["arm"]
                if self._load_gripper:
                    action_space_joints = flatten_list(
                        ["gripper_width", "gripper_max_effort", action_space_joints]
                        if self.joints["both"][0] in self.joints["hand"]
                        else [
                            action_space_joints,
                            "gripper_width",
                            "gripper_max_effort",
                        ]
                    )

        return action_space_joints

    def _create_action_space(self):
        """Create the action space based on the action space size and the action bounds.

        Returns:
            :obj:`gym.spaces.Box`: The gym action space.
        """
        # Retrieve action space joints if not supplied
        self._action_space_joints = self._get_action_space_joints()

        # Set action space bounds based on control_type
        arm_bounds_key = (
            "ee_pose"
            if self.robot_control_type == "end_effector"
            else (
                "joint_efforts"
                if self.robot_control_type == "effort"
                else "joint_positions"
            )
        )
        arm_action_bounds_low = {
            joint: val
            for joint, val in self._action_bounds[arm_bounds_key]["low"].items()
            if joint != "gripper_max_effort"
        }
        gripper_with_action_bound_low = {
            joint: val
            for joint, val in self._action_bounds["joint_positions"]["low"].items()
            if joint == "gripper_width"
        }
        gripper_max_effort_action_bound_low = {
            joint: val
            for joint, val in self._action_bounds["joint_efforts"]["low"].items()
            if joint == "gripper_max_effort"
        }
        action_bounds_low = merge_n_dicts(
            arm_action_bounds_low,
            gripper_with_action_bound_low,
            gripper_max_effort_action_bound_low,
            order=self._action_space_joints,
        )
        arm_action_bounds_high = {
            joint: val
            for joint, val in self._action_bounds[arm_bounds_key]["high"].items()
            if joint != "gripper_max_effort"
        }
        gripper_with_action_bound_high = {
            joint: val
            for joint, val in self._action_bounds["joint_positions"]["high"].items()
            if joint == "gripper_width"
        }
        gripper_max_effort_action_bound_high = {
            joint: val
            for joint, val in self._action_bounds["joint_efforts"]["high"].items()
            if joint == "gripper_max_effort"
        }
        action_bounds_high = merge_n_dicts(
            arm_action_bounds_high,
            gripper_with_action_bound_high,
            gripper_max_effort_action_bound_high,
            order=self._action_space_joints,
        )
        action_bound_low_filtered = np.array(
            [
                val
                for key, val in action_bounds_low.items()
                if key in self._action_space_joints
            ]
        )
        action_bound_high_filtered = np.array(
            [
                val
                for key, val in action_bounds_high.items()
                if key in self._action_space_joints
            ]
        )

        # Create action space
        return spaces.Box(
            action_bound_low_filtered,
            action_bound_high_filtered,
            shape=action_bound_low_filtered.shape,
            dtype="float32",
        )

    def _set_panda_configuration(self, joint_names, joint_positions):
        """Set the panda robot to a given configuration.

        Args:
            joint_positions (list): The desired joint positions.
            joint_names (list): The joint names for which you want to set the joint
                position.

        Returns:
            bool: Whether the panda configuration was successfully set.

        .. info::
            This method was implemented to ensure that the joint positions stay inside
            the joint_limits when Gazebo's ``set_model_configuration`` service is used.
            For more information, see
            `https://github.com/frankaemika/franka_ros/issues/225 <https://github.com/frankaemika/franka_ros/issues/225>`_.
        """  # noqa: E501
        retval = self.gazebo.set_model_configuration(
            model_name="panda",
            joint_names=joint_names,
            joint_positions=joint_positions,
        )

        # Reset the franka joint positions
        # NOTE: Needed because https://github.com/frankaemika/franka_ros/issues/225
        if hasattr(self, "_franka_gazebo_reset_joint_states_srv"):
            resp = self._franka_gazebo_reset_joint_states_srv.call(
                ResetJointStatesRequest()
            )
            retval = resp.success
        else:
            retval = False

        return retval

    @property
    def ee_pose(self):
        """Returns the ee pose while taking the `reward_frame_offset` into account when
        it has been set in the environment config.

        Returns:
            :obj:`geometry_msgs.msg.PoseStamped`: The stamped ee pose.
        """
        if self._ee_frame_offset and sum(list(self._ee_frame_offset.values())) != 1.0:
            try:
                ee_to_world_transform = self.tf_buffer.lookup_transform(
                    "world", self._ee_link, rospy.Time(0)
                )
            except (
                LookupException,
                ConnectivityException,
                ExtrapolationException,
            ) as e:
                rospy.logerr(
                    "Shutting down '{}' since the {}.".format(
                        rospy.get_name(), lower_first_char(e.args[0])
                    )
                )
                sys.exit(0)
            rew_frame_offset_pose_stamped = PoseStamped(
                header=Header(frame_id=self._ee_link, stamp=rospy.Time.now()),
                pose=Pose(
                    position=Vector3(
                        x=self._ee_frame_offset["x"],
                        y=self._ee_frame_offset["y"],
                        z=self._ee_frame_offset["z"],
                    ),
                    orientation=normalize_quaternion(
                        Quaternion(
                            x=self._ee_frame_offset["rx"],
                            y=self._ee_frame_offset["ry"],
                            z=self._ee_frame_offset["rz"],
                            w=self._ee_frame_offset["rw"],
                        )
                    ),
                ),
            )
            return tf2_geometry_msgs.do_transform_pose(
                rew_frame_offset_pose_stamped, ee_to_world_transform
            )
        else:
            try:
                return self.get_ee_pose()
            except EePoseLookupError as e:
                rospy.logerr(
                    "Shutting down '{}' since the {}.".format(
                        rospy.get_name(), lower_first_char(e.args[0])
                    )
                )
                sys.exit(0)

    ################################################
    # Overload Robot env virtual methods ###########
    ################################################
    # NOTE: Methods that need to be implemented as they are called by the robot and
    # gazebo environments.
    def _compute_reward(self, observations, done):
        """Compute the reward.

        Args:
            observations (dict): Dictionary containing the observations.
            done (bool): Boolean specifying whether an episode is terminated.

        Returns:
            :obj:`numpy.float32`: Reward that is received by the agent.

        .. attention::
            The ``done`` argument is not used for computing the reward in this task
            environment.
        """
        # Calculate the rewards based on the distance from the goal
        d = self._goal_distance(observations["achieved_goal"], self.goal)
        if self._reward_type == "sparse":
            if self._collision_penalty != 0.0 and self.in_collision:
                reward = np.float32(-1.0)
            else:
                reward = -(d > self._distance_threshold).astype(np.float32)
            self._step_debug_logger("=Reward info=")
            self._step_debug_logger("Reward type: Non sparse")
            self._step_debug_logger("Goal: %s", self.goal)
            self._step_debug_logger("Achieved goal: %s", observations["achieved_goal"])
            self._step_debug_logger("Perpendicular distance: %s", d)
            self._step_debug_logger("Threshold: %s", self._distance_threshold)
            self._step_debug_logger(
                "Received reward: %s",
                reward,
            )
            return reward
        else:
            self._step_debug_logger("=Reward info=")
            self._step_debug_logger("Reward type: Sparse")
            self._step_debug_logger("Goal: %s", self.goal)
            self._step_debug_logger("Achieved goal: %s", observations["achieved_goal"])
            self._step_debug_logger("Perpendicular distance: %s", d)
            self._step_debug_logger("Threshold: %s", self._distance_threshold)
            self._step_debug_logger("Received reward: %s", -d)
            reward = -d
            if self._collision_penalty != 0.0 and self.in_collision:
                reward -= np.float64(self._collision_penalty)
            return reward

    def _get_obs(self):
        """Get robot state observation.

        Returns:
            tuple containing:

                - observation array (:obj:`list`):
                    List of observations:
                        - End effector x position
                        - End effector y position
                        - End effector z position
                        - End effector joints positions
                        - End effector joints velocities
                - achieved_goal (:obj:`object`): The goal that was achieved during
                    execution.
                - desired_goal (:obj:`object`): The desired goal that we asked the agent
                    to attempt to achieve.
                - info (:obj:`dict`): An info dictionary with additional information.
        """
        ee_pose = self.ee_pose
        ee_position = np.array(
            [
                ee_pose.pose.position.x,
                ee_pose.pose.position.y,
                ee_pose.pose.position.z,
            ]
        )
        achieved_goal = ee_position

        robot_qpos, robot_qvel = self._robot_get_obs()
        ee_state = [
            pos
            for pos, name in zip(robot_qpos, self.joint_states.name)
            if "panda_finger" in name
        ]
        ee_vel = [
            vel
            for vel, name in zip(robot_qvel, self.joint_states.name)
            if "panda_finger" in name
        ]

        obs = np.concatenate(
            [
                ee_position,
                ee_state,
                ee_vel,
            ]
        )
        return {
            "observation": obs.copy(),
            "achieved_goal": achieved_goal.copy(),
            "desired_goal": self.goal.copy(),
            "info": {},
        }

    def _set_action(self, action):
        """Take robot action.

        Args:
            action (numpy.ndarray): List containing joint or ee action commands.
        """
        # Throw error and shutdown if action space is not the right size
        if not action.shape == self.action_space.shape:
            rospy.logerr(
                f"Shutting down '{rospy.get_name()}' since the shape of the supplied "
                f"action {action.shape} while the gym action space has shape "
                f"{self.action_space.shape}."
            )
            sys.exit(0)

        # Send action commands to the controllers based on control type
        action_dict = dict(zip(self._action_space_joints, action))
        self._step_debug_logger("=Action set info=")
        self._step_debug_logger("Action that is set:")
        self._step_debug_logger(list(action_dict.values()))
        if self.robot_control_type in ["end_effector", "trajectory"]:
            gripper_with = action_dict.pop("gripper_width", None)
            gripper_max_effort = action_dict.pop("gripper_max_effort", None)
            if self.robot_control_type == "end_effector":
                self.set_ee_pose(action_dict)
                if self._load_gripper and not self._block_gripper:
                    self.set_gripper_width(
                        gripper_with,
                        wait=self._hand_wait,
                        max_effort=gripper_max_effort,
                    )
            else:
                self.set_arm_joint_trajectory(action_dict, wait=self._arm_wait)
                if self._load_gripper and not self._block_gripper:
                    self.set_gripper_width(
                        gripper_with,
                        wait=self._hand_wait,
                        max_effort=gripper_max_effort,
                    )
        else:
            if self._load_gripper and self._block_gripper:
                action_dict.pop("gripper_width", None)
                action_dict.pop("gripper_max_effort", None)
            self.set_joint_commands(
                action_dict, arm_wait=self._arm_wait, hand_wait=self._hand_wait
            )

    def _is_done(self, observations):
        """Check if task is done.

        Args:
            observations (dict): Dictionary containing the observations

        Returns:
            bool: Boolean specifying whether the episode is done (e.i. distance to the
                goal is within the distance threshold, robot has fallen etc.).
        """
        # Check if gripper is within range of the goal
        d = self._goal_distance(observations["achieved_goal"], self.goal)
        is_done = (d < self._distance_threshold).astype(np.float32)

        self._step_debug_logger("=Task is done info=")
        if self._target_hold:
            if is_done:
                self._is_done_samples += 1
                if self._is_done_samples < self._hold_samples:
                    is_done = False
                    self._step_debug_logger(
                        f"Agent spend '{self._is_done_samples}' within target position."
                    )
                else:
                    self._step_debug_logger("Task is done.")
                    self._is_done_samples = 0
            else:
                self._step_debug_logger("Task is not done.")
                self._is_done_samples = 0
        else:
            if is_done:
                self._step_debug_logger("Task is done.")
            else:
                self._step_debug_logger("Task is not done.")
        return is_done

    def _sample_goal(self):
        """Sample a random goal position.

        Returns:
            :obj:`geometry_msgs.PoseStamped`: A goal pose.
        """
        rospy.logdebug("Sampling goal.")
        if self._target_sampling_strategy == "global":
            goal = self.np_random.uniform(
                [
                    self._target_sampling_bounds["x_min"],
                    self._target_sampling_bounds["y_min"],
                    self._target_sampling_bounds["z_min"],
                ],
                [
                    self._target_sampling_bounds["x_max"],
                    self._target_sampling_bounds["y_max"],
                    self._target_sampling_bounds["z_max"],
                ],
                size=3,
            )
        elif self._target_sampling_strategy == "local":  # Rel to current EE pose
            try:
                cur_ee_pose = self.get_ee_pose()
                cur_ee_position = np.array(
                    [
                        cur_ee_pose.pose.position.x,
                        cur_ee_pose.pose.position.y,
                        cur_ee_pose.pose.position.z,
                    ]
                )
            except EePoseLookupError:
                rospy.logerr(
                    f"Shutting down '{rospy.get_name()}' since the current end "
                    "effector pose which is needed for sampling the goals could "
                    "not be retrieved."
                )
                sys.exit(0)

            # Sample goal relative to end effector pose
            goal = cur_ee_position + self.np_random.uniform(
                [
                    self._target_sampling_bounds["x_min"],
                    self._target_sampling_bounds["y_min"],
                    self._target_sampling_bounds["z_min"],
                ],
                [
                    self._target_sampling_bounds["x_max"],
                    self._target_sampling_bounds["y_max"],
                    self._target_sampling_bounds["z_max"],
                ],
                size=cur_ee_position.shape,
            )
        elif self._target_sampling_strategy == "fixed":
            goal = np.array(list(self._fixed_target_pose.values()))
        else:  # Thrown error if goal could not be sampled
            rospy.logerr(
                f"Shutting down '{rospy.get_name()}' since no goal could be sampled "
                f"as '{self._target_sampling_strategy}' is not a valid goal sampling "
                "strategy. Options are 'global' and 'local'."
            )
            sys.exit(0)

        # Make sure the goal is always within the sampling region
        if self._target_sampling_strategy != "fixed":
            goal = self._clip_goal_position(goal)

        rospy.logdebug("Goal: %s" % goal)

        # Apply offset and return goal
        goal += np.array(
            [
                self._target_offset["x"],
                self._target_offset["y"],
                self._target_offset["z"],
            ]
        )
        return goal

    def _visualize_goal(self, offset=[0.0, 0.0, 0.0]):
        """Visualize RViz goal marker.

        Args:
            offset (list): Position offset for visualizing the goal.
        """
        goal = self.goal + offset
        goal_marker_msg = TargetMarker(x=goal[0], y=goal[1], z=goal[2], id=3)
        self._target_pose_marker_pub.publish(goal_marker_msg)

    def _set_init_pose(self):  # noqa: C901
        """Sets the Robot in its (random) initial pose.

        Returns:
            bool: Boolean specifying whether the initial pose was successfully set.

        .. info::
           The pose is sampled based on the ``pose_sampling_type`` variable set in the
           task environment configuration file. Options are ``end_effector_pose``,
           which creates a random pose by sampling EE poses, and ``joint_positions``,
           which makes a random pose by sampling joint positions. These methods respect
           the ``bounds`` set in the task environment config.
        """
        if self._reset_init_pose:
            if self._random_init_pose:  # Use random initial model configuration
                if (
                    self._pose_sampling_type == "joint_positions"
                ):  # Sample random joint positions
                    rospy.logdebug("Retrieve random joint positions.")
                    random_joint_positions = self._get_random_joint_positions()
                    if random_joint_positions:
                        if self._load_gripper:
                            random_joint_positions["gripper_width"] = (
                                split_pose_dict(self._init_pose)[1]["gripper_width"]
                                if self._block_gripper
                                else random_joint_positions.pop(
                                    self.joints["hand"][0], 0.0
                                )
                                * 2
                            )
                            random_joint_positions = {
                                joint: position
                                for joint, position in random_joint_positions.items()
                                if joint not in self.joints["hand"]
                            }
                else:  # Sample random EE poses
                    if (
                        self._randomize_first_episode or self.episode_num != 0
                    ):  # Retrieve random EE pose
                        rospy.logdebug("Retrieve random EE pose.")
                        self._set_panda_configuration(
                            joint_names=self.joints["both"],
                            joint_positions=PANDA_REST_CONFIGURATION[
                                : len(self.joints["both"])
                            ],
                        )  # NOTE: Done as joint conflicts might prevent planning
                        (
                            random_ee_pose,
                            random_joint_positions,
                        ) = self._get_random_ee_pose()
                    else:  # Use fixed initial arm/hand initial EE pose
                        rospy.logdebug("Retrieving initial EE pose.")
                        random_ee_pose = split_pose_dict(self._init_pose)[0]
                        random_joint_positions = self.get_ee_pose_joint_config(
                            random_ee_pose
                        )

                    # Apply init pose offset
                    if random_ee_pose and sum(self._init_pose_offset.values()) != 0.0:
                        rospy.logdebug("Applying offset to initial pose.")
                        random_ee_pose["x"] += self._init_pose_offset["x"]
                        random_ee_pose["y"] += self._init_pose_offset["y"]
                        random_ee_pose["z"] += self._init_pose_offset["z"]

                        # Retrieve model configuration for the new EE pose
                        ee_pose_joint_positions = self.get_ee_pose_joint_config(
                            random_ee_pose
                        )
                        if not ee_pose_joint_positions:
                            rospy.logwarn(
                                "Could not retrieve a model configuration that relates "
                                "to the EE pose with the offset. As a result the model "
                                "configuration for the EE pose without the offset is "
                                "used."
                            )
                        else:
                            random_joint_positions = ee_pose_joint_positions

                    # Retrieve random gripper width
                    if self._load_gripper:
                        random_joint_positions["gripper_width"] = (
                            split_pose_dict(self._init_pose)[1]["gripper_width"]
                            if self._block_gripper
                            else self._get_random_joint_positions().pop(
                                self.joints["hand"][0], 0.0
                            )
                            * 2
                        )
            else:  # Use fixed initial pose
                if self._pose_sampling_type == "joint_positions":
                    rospy.logdebug("Retrieving initial joint positions.")
                    _, self._init_model_configuration = split_pose_dict(self._init_pose)
                    if not self._load_gripper:
                        self._init_model_configuration.pop("gripper_width", None)
                else:
                    rospy.logdebug("Retrieving initial EE pose.")
                    random_joint_positions = self.get_ee_pose_joint_config(
                        split_pose_dict(self._init_pose)[0]
                    )
                    if random_joint_positions:
                        if self._load_gripper:
                            random_joint_positions["gripper_width"] = split_pose_dict(
                                self._init_pose
                            )[1]["gripper_width"]
                    else:
                        random_joint_positions = dict(
                            zip(
                                self.joints["both"],
                                PANDA_REST_CONFIGURATION[: len(self.joints["both"])],
                            )
                        )
                        rospy.logwarn_once(
                            "No valid joint positions could be retrieved for the EE "
                            "pose specified in the 'init_pose' field of the task "
                            "configuration file. As a "
                            "result the fallback model configuration was used "
                            "'{}'.".format(random_joint_positions)
                        )

            # Check if a valid model configuration was found
            if random_joint_positions:
                self._init_model_configuration = random_joint_positions
            else:
                if self._init_model_configuration:
                    rospy.logwarn(
                        "No valid random {} could be retrieved. As a result the "
                        "initial model configuration of the previous episode was "
                        "used.".format(
                            "joint positions"
                            if self._pose_sampling_type == "joint_positions"
                            else "EE pose"
                        )
                    )
                else:
                    self._init_model_configuration = dict(
                        zip(
                            self.joints["both"],
                            PANDA_REST_CONFIGURATION[: len(self.joints["both"])],
                        )
                    )
                    rospy.logwarn(
                        "No valid random {} could be retrieved. As a result the "
                        "fallback model configuration was used '{}'.".format(
                            "joint positions"
                            if self._pose_sampling_type == "joint_positions"
                            else "EE pose",
                            self._init_model_configuration,
                        )
                    )

            # Convert gripper width to joint positions
            if self._load_gripper:
                self._init_model_configuration = (
                    gripper_width_2_finger_joints_positions(
                        self._init_model_configuration, self.joints["hand"]
                    )
                )

            # Set initial model configuration and return result
            # NOTE: Here two modes can be used: MoveIT or Gazebo's
            # 'set_model_configuration' service.
            if not self._moveit_init_pose_control:  # Use Gazebo service
                rospy.loginfo("Setting initial robot pose.")
                init_pose_retval = self._set_panda_configuration(
                    joint_names=self._init_model_configuration.keys(),
                    joint_positions=self._init_model_configuration.values(),
                )
                if not init_pose_retval:
                    rospy.logwarn("Setting initial robot pose failed.")
                return init_pose_retval
            else:  # Use MoveIt
                if hasattr(self, "_moveit_set_joint_positions_srv"):
                    prev_control_type = self.robot_control_type
                    self.robot_control_type = "trajectory"
                    resp = self._moveit_set_joint_positions_srv.call(
                        pg_srv.SetJointPositionsRequest(
                            joint_names=self._init_model_configuration.keys(),
                            joint_positions=self._init_model_configuration.values(),
                            wait=True,
                        )
                    )
                    self.robot_control_type = prev_control_type
                    if not resp.success:
                        rospy.logwarn("Setting initial robot pose failed.")
                    return resp.success
                else:
                    rospy.logwarn(
                        "The initial pose failed since the {} service was not "
                        "available.".format(MOVEIT_SET_JOINT_POSITIONS_TOPIC)
                    )
                    return False
        else:
            return True

    def _init_env_variables(self):
        """Inits variables needed to be initialized each time we reset at the start
        of an episode.
        """
        # Sample and visualize goal
        self.goal = self._sample_goal()
        if self._visualize_target:
            self._visualize_goal()
