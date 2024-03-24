"""Robot environment for the
`Panda Emika Franka Simulation <https://ros-planning.github.io/moveit_tutorials/doc/gazebo_simulation/gazebo_simulation.html>`_.

.. note::
    The panda robot environment contains two methods of controlling the robot: ``DIRECT``
    control (Default) and ``PROXY`` based control. Initially, I abstracted all the panda
    control logic away in the :panda-gazebo:`panda_gazebo <>` package and made it
    available through services. Later, however, I found that the service calls slowed
    down the control. I then added the DIRECT control method, which directly publishes
    the control commands on the controller ``command`` topic. This control method made
    the training loop two times faster. Currently, the ``DIRECT`` mode is only available
    for the ``position`` and ``effort`` control. Other control methods like
    ``trajectory`` and ``end_effector`` control will use the PROXY based method.
"""  # noqa: E501

import copy
from datetime import datetime
from itertools import compress

import actionlib
import numpy as np
import rospy
import tf2_ros
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from geometry_msgs.msg import (
    Pose,
    Quaternion,
    Vector3,
    TransformStamped,
    Transform,
    Point,
)
from tf.transformations import quaternion_inverse, quaternion_multiply
from ros_gazebo_gym.common.helpers import (
    action_server_exists,
    flatten_list,
    get_orientation_euler,
    is_sublist,
    list_2_human_text,
    lower_first_char,
    normalize_quaternion,
    suppress_stderr,
)
from ros_gazebo_gym.core.helpers import get_log_path, ros_exit_gracefully
from ros_gazebo_gym.core.lazy_importer import LazyImporter
from ros_gazebo_gym.core.ros_launcher import ROSLauncher
from ros_gazebo_gym.exceptions import EePoseLookupError, EeRpyLookupError
from ros_gazebo_gym.robot_envs.helpers import (
    remove_gripper_commands_from_joint_commands_msg,
)
from ros_gazebo_gym.robot_gazebo_goal_env import RobotGazeboGoalEnv
from rospy.exceptions import ROSException, ROSInterruptException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray, Header
from urdf_parser_py.urdf import URDF
from tf2_geometry_msgs import PoseStamped
from tf2_ros import (
    StaticTransformBroadcaster,
)

# Specify topics and connection timeouts.
CONNECTION_TIMEOUT = 5  # Timeout for connecting to services or topics.
GAZEBO_SIM_CONNECTION_TIMEOUT = 60  # Timeout for waiting for gazebo to be launched.
MOVEIT_SET_EE_POSE_TOPIC = "panda_moveit_planner_server/panda_arm/set_ee_pose"
MOVEIT_GET_EE_POSE_JOINT_CONFIG_TOPIC = (
    "panda_moveit_planner_server/panda_arm/get_ee_pose_joint_config"
)
LOCK_UNLOCK_TOPIC = "lock_unlock_panda_joints"
GET_CONTROLLED_JOINTS_TOPIC = "panda_control_server/get_controlled_joints"
SET_JOINT_COMMANDS_TOPIC = "panda_control_server/set_joint_commands"
SET_GRIPPER_WIDTH_TOPIC = "panda_control_server/panda_hand/set_gripper_width"
SET_JOINT_TRAJECTORY_TOPIC = "panda_control_server/panda_arm/follow_joint_trajectory"
FRANKA_GRIPPER_COMMAND_TOPIC = "franka_gripper/gripper_action"
JOINT_STATES_TOPIC = "joint_states"
FRANKA_STATES_TOPIC = "franka_state_controller/franka_states"
GET_PANDA_EE_FRAME_TRANSFORM_TIMEOUT = (
    1  # Timeout for retrieving the panda EE frame transform.  # noqa: E501
)

# Other script variables.
AVAILABLE_CONTROL_TYPES = [
    "trajectory",
    "position",
    "effort",
    "end_effector",
]
PANDA_JOINTS_FALLBACK = {
    "arm": [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ],
    "hand": ["panda_finger_joint1", "panda_finger_joint2"],
}  # NOTE: Used when the joints can not be determined.
ARM_POSITION_CONTROLLER = "panda_arm_joint_position_controller"
ARM_EFFORT_CONTROLLER = "panda_arm_joint_effort_controller"
GRASP_FORCE = 10  # Default panda gripper force. Panda force information: {Continuous force: 70N, max_force: 140 N}.  # noqa: E501
ARM_CONTROL_WAIT_TIMEOUT = 5  # Default arm control wait timeout [s].
ARM_JOINT_POSITION_WAIT_THRESHOLD = 0.07  # Threshold used for determining whether a joint position is reached (i.e. 0.01 rad per joint).  # noqa: E501
ARM_JOINT_EFFORT_WAIT_THRESHOLD = 7  # Threshold used for determining whether a joint position is reached (i.e. 1 N per joint).  # noqa: E501
ARM_JOINT_VELOCITY_WAIT_THRESHOLD = 0.07  # Threshold used for determining whether the joint velocity is zero (i.e. 1rad/s per joint).  # noqa: E501


class PandaEnv(RobotGazeboGoalEnv):
    """Used for controlling the panda robot and retrieving sensor data.

    To check any topic we need to have the simulations running, we need to do two
    things:

        1. Un-pause the simulation: without that the stream of data doesn't flow. This
           is for simulations that are paused for whatever the reason.
        2. If the simulation was running already for some reason, we need to reset the
           controllers.

    This has to do with the fact that some plugins with tf, don't understand the
    reset of the simulation and need to be reset to work properly.

    Attributes:
        robot_name_space (str): The robot name space.
        reset_controls (bool): Whether the controllers are reset when the simulation is
            reset.
        robot_EE_link (str): The link used for the end effector control.
        ee_frame_offset (dict): Dictionary containing the used end effector offset.
        load_gripper (bool): Whether the gripper was loaded.
        lock_gripper (bool): Whether the gripper should be locked (i.e. not move).
        joint_states (:obj:`sensor_msgs.msg.JointState`): The current joint states.
        franka_states (:obj:`franka_msgs.msg.FrankaState`): The current franka states.
            These give robot specific information about the panda robot.
        tf_buffer (:obj:`tf2_ros.buffer.Buffer`): Tf buffer object can be used to
            request transforms.
        panda_gazebo (:obj:`ros_gazebo_gym.core.LazyImporter`): Lazy importer for the
            :panda-gazebo:`panda_gazebo <>` package.
        franka_msgs (:obj:`ros_gazebo_gym.core.LazyImporter`): Lazy importer for the
            :franka-ros:`franka_msgs <tree/develop/franka_msgs>` package.
    """

    def __init__(  # noqa: C901
        self,
        robot_name_space="",
        robot_EE_link="panda_link8",
        ee_frame_offset=None,
        load_gripper=True,
        lock_gripper=False,
        grasping=False,
        control_type="effort",
        reset_robot_pose=True,
        workspace_path=None,
        log_reset=True,
        visualize=None,
    ):
        """Initializes a new Panda Robot environment.

        Args:
            robot_name_space (str, optional): The namespace the robot is on. Defaults to
                ``""``.
            robot_EE_link (str, optional): Robot end effector link name. Defaults to
                ``panda_link8``.
            ee_frame_offset (dict, optional): Dictionary containing the end effector
                offset. Used when retrieving and setting the end effector pose. Defaults
                to ``None``.
            load_gripper (bool, optional): Whether we want to load the parallel-jaw
                gripper. Defaults to ``True``.
            lock_gripper (bool, optional): Whether we want to lock the parallel-jaw
                gripper (i.e. not move). Defaults to ``False``.
            grasping (bool, optional): Whether we want to use the gripper for grasping.
                If ``True`` and `gripper_max_effort` is unset, applies 10N effort.
                Defaults to False.
            control_Type (str, optional): The type of control you want to use for the
                panda robot (i.e. hand and arm). Options are: ``trajectory``,
                ``position``, ``effort`` or ``end_effector``. Defaults to ``effort``.
            reset_robot_pose (bool, optional): Boolean specifying whether to reset the
                robot pose when the simulation is reset. Defaults to ``True``.
            workspace_path (str, optional): The path of the workspace in which the
                panda_gazebo package should be found. Defaults to ``None``.
            log_reset (bool, optional): Whether we want to print a log statement when
                the world/simulation is reset. Defaults to ``True``.
            visualize (bool, optional): Whether you want to show the RViz visualization.
                Defaults to ``None`` meaning the task configuration file values will be
                used.
        """
        rospy.logwarn("Initialize PandaEnv robot environment...")

        # Initialize lazy imported modules.
        # NOTE: Used because if not yet installed these packages will be installed later
        # and can therefore not be imported at the top of the file.
        self.panda_gazebo = LazyImporter("panda_gazebo")
        self.franka_msgs = LazyImporter("franka_msgs")

        # Set initial robot env variables.
        self.robot_name_space = robot_name_space
        self.reset_controls = True
        self.robot_EE_link = robot_EE_link
        self.load_gripper = load_gripper
        self.lock_gripper = lock_gripper
        self._ee_frame_offset_dict = ee_frame_offset
        self._ros_shutdown_requested = False
        self._connection_timeout = CONNECTION_TIMEOUT
        self._joint_traj_action_server_default_step_size = 1
        self._grasping = grasping
        self._direct_control = (
            True if not hasattr(self, "_direct_control") else self._direct_control
        )
        self._log_step_debug_info = (
            False
            if not hasattr(self, "_log_step_debug_info")
            else self._log_step_debug_info
        )
        self._joint_lock_client_connected = False
        self._get_controlled_joints_client_connected = False
        self._moveit_set_ee_pose_client_connected = False
        self._moveit_get_ee_pose_joint_config_client_connected = False
        self._arm_joint_traj_control_client_connected = False
        self._set_joint_commands_client_connected = False
        self._set_gripper_width_client_connected = False
        self._fetched_joints = False
        self._ros_is_shutdown = False
        self._last_gripper_goal = None
        self.__robot_control_type = control_type.lower()
        self.__joints = {}
        self.__in_collision = False
        self.__locked_joints = []

        # Thrown control warnings.
        if self._direct_control and self.robot_control_type in [
            "trajectory",
            "end_effector",
        ]:
            rospy.logwarn(
                "Direct control variable 'direct_control' was ignored as it "
                f"is not implemented for '{self.robot_control_type}' control."
            )
        if self.robot_control_type == "position":
            rospy.logwarn(
                "Position control is experimental and not yet fully implemented. "
                "See https://github.com/rickstaa/panda-gazebo/issues/12."
            )

        # Wait for the simulation to be started.
        simulation_check_timeout_time = rospy.get_rostime() + rospy.Duration(
            GAZEBO_SIM_CONNECTION_TIMEOUT
        )
        while (
            not rospy.is_shutdown()
            and rospy.get_rostime() < simulation_check_timeout_time
        ):
            if any(
                [
                    "/gazebo" in topic
                    for topic in flatten_list(rospy.get_published_topics())
                ]
            ):
                break
            else:
                rospy.logwarn_once("Waiting for the Gazebo simulation to be started...")
        else:
            if not rospy.is_shutdown():
                err_msg = (
                    f"Shutting down '{rospy.get_name()}' since the Panda Gazebo "
                    "simulation was not started within the set timeout period of "
                    f"{GAZEBO_SIM_CONNECTION_TIMEOUT} seconds."
                )
                ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)

        # Validate requested control type.
        if self.robot_control_type not in AVAILABLE_CONTROL_TYPES:
            err_msg = (
                f"Shutting down '{rospy.get_name()}' because control type "
                f"'{control_type}' that was specified is invalid. Please use one of "
                "the following robot control types and try again: "
            )
            for ctrl_type in AVAILABLE_CONTROL_TYPES:
                err_msg += f"\n - {ctrl_type}"
            ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)
        else:
            rospy.logwarn(f"Panda robot is controlled using '{control_type}' control.")

        # Launch the ROS launch that spawns the robot into the world.
        control_type_group = (
            "trajectory"
            if self.robot_control_type == "end_effector"
            else self.robot_control_type
        )  # NOTE: Ee control uses the trajectory controllers.
        launch_log_file = str(
            get_log_path().joinpath(
                "put_robot_in_world_launch_{}.log".format(
                    datetime.now().strftime("%d_%m_%Y_%H_%M_%S"),
                )
            )
            if (
                hasattr(self, "_roslaunch_log_to_console")
                and not self._roslaunch_log_to_console
            )
            else None
        )
        show_rviz = (
            visualize
            if visualize is not None
            else (self._load_rviz if hasattr(self, "_load_rviz") else True)
        )
        ROSLauncher.launch(
            package_name="panda_gazebo",
            launch_file_name="put_robot_in_world.launch",
            workspace_path=workspace_path,
            log_file=launch_log_file,
            critical=True,
            rviz=show_rviz,
            load_gripper=self.load_gripper,
            disable_franka_gazebo_logs=True,
            rviz_file=self._rviz_file if hasattr(self, "_rviz_file") else "",
            end_effector=self.robot_EE_link,
            max_velocity_scaling_factor=(
                self._max_velocity_scaling_factor
                if hasattr(self, "_max_velocity_scaling_factor")
                else ""
            ),
            max_acceleration_scaling_factor=(
                self._max_acceleration_scaling_factor
                if hasattr(self, "_max_acceleration_scaling_factor")
                else ""
            ),
            control_type=control_type_group,
        )

        # Add ros shutdown hook.
        rospy.on_shutdown(self._ros_shutdown_hook)

        ########################################
        # Initiate gazebo environment ##########
        ########################################
        # NOTE: In this env we don't supply the controllers_list but let the
        # ControllersConnection class determine them based on the running controllers.
        super(PandaEnv, self).__init__(
            robot_name_space=self.robot_name_space,
            reset_controls=self.reset_controls,
            reset_robot_pose=reset_robot_pose,
            reset_world_or_sim="WORLD",
            log_reset=log_reset,
            pause_simulation=(
                self._pause_after_step if hasattr(self, "_pause_after_step") else False
            ),
            publish_rviz_training_info_overlay=(
                self._load_rviz if hasattr(self, "_load_rviz") else True
            ),
        )

        ########################################
        # Initialize sensor topics #############
        ########################################

        # Create publishers.
        rospy.logdebug("Creating publishers.")
        self._in_collision_pub = rospy.Publisher(
            "/ros_gazebo_gym/in_collision", Float32, queue_size=1, latch=True
        )

        # Create joint state and franka state subscriber.
        rospy.logdebug("Connecting to sensors.")
        rospy.Subscriber(
            f"{self.robot_name_space}/{JOINT_STATES_TOPIC}",
            JointState,
            self._joint_states_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            FRANKA_STATES_TOPIC,
            self.franka_msgs.msg.FrankaState,
            self._franka_states_cb,
            queue_size=1,
        )

        # Create transform listener.
        rospy.logdebug("Creating tf2 buffer.")
        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        ########################################
        # Validate environment arguments #######
        ########################################
        rospy.logdebug("Validating environment arguments.")

        # Retrieve the robot model from the parameter server.
        rospy.logdebug("Retrieving robot model from the parameter server.")
        try:
            with suppress_stderr():
                self._robot = URDF.from_parameter_server()
        except KeyError as e:
            if e.args[0] == "robot_description":
                err_msg = (
                    "Failed to retrieve robot model from the parameter server. "
                    "Please make sure the robot model is loaded on the parameter server "
                    "and try again."
                )
                ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)
            raise e

        # Ensure specified end-effector link exists.
        rospy.logdebug("Validating end-effector link.")
        if not self.ee_link_exists:
            err_msg = (
                f"The specified end-effector link '{self.robot_EE_link}' does not "
                "exist. Please make sure the end-effector link exists and try again."
            )
            ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)

        # Retrieve ee frame offset.
        rospy.logdebug("Validating end-effector frame offset.")
        if self._ee_frame_offset_dict is None:
            rospy.logdebug(
                f"Offset not specified. Using transform between '{self.robot_EE_link}' "
                "and 'panda_EE' as offset."
            )
            try:
                panda_ee_frame_offset = self._get_frame_pose(
                    parent_frame=self.robot_EE_link,
                    child_frame="panda_EE",
                    timeout=GET_PANDA_EE_FRAME_TRANSFORM_TIMEOUT,
                )
                self._ee_frame_offset_dict = {
                    "x": panda_ee_frame_offset.pose.position.x,
                    "y": panda_ee_frame_offset.pose.position.y,
                    "z": panda_ee_frame_offset.pose.position.z,
                    "rx": panda_ee_frame_offset.pose.orientation.x,
                    "ry": panda_ee_frame_offset.pose.orientation.y,
                    "rz": panda_ee_frame_offset.pose.orientation.z,
                    "rw": panda_ee_frame_offset.pose.orientation.w,
                }
            except TimeoutError as e:
                ros_exit_gracefully(shutdown_msg=e.args[0], exit_code=1)

        ########################################
        # Create publishers ####################
        ########################################

        # Create static transform broadcaster.
        rospy.logdebug("Creating static transform broadcaster.")
        self._static_tf_broadcaster = StaticTransformBroadcaster()

        # Publish static transform between the world and the end-effector.
        # NOTE: Used for RVIZ visualization.
        self._add_ee_frame()

        ########################################
        # Connect to control services ##########
        ########################################
        rospy.loginfo("Connecting to robot control services.")

        ################################
        # Control switcher #############
        ################################
        # NOTE: Here we use a wrapper around the 'control_manager/switch_controller'
        # provided by the 'panda_gazebo' package that knows which controllers to load
        # for each control type.
        rospy.logdebug("Creating to Panda Control switcher.")
        self._controller_switcher = (
            self.panda_gazebo.core.control_switcher.PandaControlSwitcher(
                connection_timeout=self._connection_timeout,
                robot_name_space=self.robot_name_space,
            )
        )

        ################################
        # Joint Locker #################
        ################################

        # Connect to the 'panda_gazebo' joint lock/unlock service.
        # NOTE: Used to lock specific panda arm joints.
        try:
            joint_lock_srv_topic = f"{self.robot_name_space}/{LOCK_UNLOCK_TOPIC}"
            rospy.logdebug("Connecting to '%s' service." % joint_lock_srv_topic)
            rospy.wait_for_service(
                joint_lock_srv_topic, timeout=self._connection_timeout
            )
            self._joint_lock_client = rospy.ServiceProxy(
                joint_lock_srv_topic,
                self.panda_gazebo.srv.LockJoints,
            )
            rospy.logdebug("Connected to '%s' service!" % joint_lock_srv_topic)
            self._joint_lock_client_connected = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn("Failed to connect to '%s' service!" % joint_lock_srv_topic)

        ################################
        # MoveIt Control services ######
        ################################

        # Connect to Panda control server 'get_controlled_joints' service.
        try:
            get_controlled_joints_srv_topic = (
                f"{self.robot_name_space}/{GET_CONTROLLED_JOINTS_TOPIC}"
            )
            rospy.logdebug(
                "Connecting to '%s' service." % get_controlled_joints_srv_topic
            )
            rospy.wait_for_service(
                get_controlled_joints_srv_topic,
                timeout=self._connection_timeout,
            )
            self._get_controlled_joints_client = rospy.ServiceProxy(
                get_controlled_joints_srv_topic,
                self.panda_gazebo.srv.GetControlledJoints,
            )
            rospy.logdebug(
                "Connected to '%s' service!" % get_controlled_joints_srv_topic
            )
            self._get_controlled_joints_client_connected = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % get_controlled_joints_srv_topic
            )

        # Connect to MoveIt 'set_ee_pose' topic.
        if self.robot_control_type == "end_effector":
            try:
                moveit_set_ee_pose_srv_topic = (
                    f"{self.robot_name_space}/{MOVEIT_SET_EE_POSE_TOPIC}"
                )
                rospy.logdebug(
                    "Connecting to '%s' service." % moveit_set_ee_pose_srv_topic
                )
                rospy.wait_for_service(
                    moveit_set_ee_pose_srv_topic,
                    timeout=self._connection_timeout,
                )
                self._moveit_set_ee_pose_client = rospy.ServiceProxy(
                    moveit_set_ee_pose_srv_topic, self.panda_gazebo.srv.SetEePose
                )
                rospy.logdebug(
                    "Connected to '%s' service!" % moveit_set_ee_pose_srv_topic
                )
                self._moveit_set_ee_pose_client_connected = True
            except (rospy.ServiceException, ROSException, ROSInterruptException):
                err_msg = (
                    f"Shutting down '{rospy.get_name()}' since no connection could be "
                    f"established with the '{moveit_set_ee_pose_srv_topic}' service. "
                    "This service is needed for controlling the Panda robot using the "
                    f"'{self.robot_control_type}' control type."
                )
                ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)

        # Connect to MoveIt 'get_ee_pose_joint_config' service.
        try:
            moveit_get_ee_pose_joint_config_srv_topic = (
                f"{self.robot_name_space}/{MOVEIT_GET_EE_POSE_JOINT_CONFIG_TOPIC}"
            )
            rospy.logdebug(
                "Connecting to '%s' service."
                % moveit_get_ee_pose_joint_config_srv_topic
            )
            rospy.wait_for_service(
                moveit_get_ee_pose_joint_config_srv_topic,
                timeout=self._connection_timeout,
            )
            self._moveit_get_ee_pose_joint_config_client = rospy.ServiceProxy(
                moveit_get_ee_pose_joint_config_srv_topic,
                self.panda_gazebo.srv.GetEePoseJointConfig,
            )
            rospy.logdebug(
                "Connected to '%s' service!" % moveit_get_ee_pose_joint_config_srv_topic
            )
            self._moveit_get_ee_pose_joint_config_client_connected = True
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!"
                % moveit_get_ee_pose_joint_config_srv_topic
            )

        ################################
        # Trajectory action service ####
        ################################
        if self.robot_control_type == "trajectory":
            # Connect to Joint Trajectory Panda Control (action) service.
            set_joint_trajectory_action_srv_topic = (
                f"{self.robot_name_space}/{SET_JOINT_TRAJECTORY_TOPIC}"
            )
            rospy.logdebug(
                "Connecting to '%s' action service."
                % set_joint_trajectory_action_srv_topic
            )
            if action_server_exists(
                set_joint_trajectory_action_srv_topic
            ):  # Check if exists.
                # Connect to robot control action server.
                self._arm_joint_traj_control_client = actionlib.SimpleActionClient(
                    set_joint_trajectory_action_srv_topic,
                    self.panda_gazebo.msg.FollowJointTrajectoryAction,
                )
                # Waits until the action server has started up.
                retval = self._arm_joint_traj_control_client.wait_for_server(
                    timeout=rospy.Duration(self._connection_timeout)
                )
                if retval:
                    self._arm_joint_traj_control_client_connected = True

            # Shutdown if not connected.
            if not self._arm_joint_traj_control_client_connected:
                err_msg = (
                    f"Shutting down '{rospy.get_name()}' since no connection could be "
                    f"established with  the '{set_joint_trajectory_action_srv_topic}' "
                    "service. This service is needed for controlling the Panda robot "
                    f"using the '{self.robot_control_type}' control type."
                )
                ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)

        ################################
        # Panda control services #######
        ################################

        # Connect to arm control services/topics.
        if control_type_group != "trajectory":
            if not self._direct_control:  # Use 'panda_gazebo' services.
                # Connect to Panda Control server 'set_joint_commands' service.
                try:
                    set_joint_commands_srv_topic = (
                        f"{self.robot_name_space}/{SET_JOINT_COMMANDS_TOPIC}"
                    )
                    rospy.logdebug(
                        "Connecting to '%s' service." % set_joint_commands_srv_topic
                    )
                    rospy.wait_for_service(
                        set_joint_commands_srv_topic,
                        timeout=self._connection_timeout,
                    )
                    self._set_joint_commands_client = rospy.ServiceProxy(
                        set_joint_commands_srv_topic,
                        self.panda_gazebo.srv.SetJointCommands,
                    )
                    rospy.logdebug(
                        "Connected to '%s' service!" % set_joint_commands_srv_topic
                    )
                    self._set_joint_commands_client_connected = True
                except (rospy.ServiceException, ROSException, ROSInterruptException):
                    err_msg = (
                        f"Shutting down '{rospy.get_name()}' since no connection could "
                        f"be established with the '{set_joint_commands_srv_topic}' "
                        "service. This service is needed for controlling the arm in "
                        "'PROXY' control mode."
                    )
                    ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)
            else:  # Directly publish control commands to controller topics.
                if self.robot_control_type == "position":
                    # Create arm joint position control publisher.
                    self._arm_joint_position_pub = rospy.Publisher(
                        "%s/command" % (ARM_POSITION_CONTROLLER),
                        Float64MultiArray,
                        queue_size=10,
                    )
                else:
                    # Create arm joint effort control publisher.
                    self._arm_joint_effort_pub = rospy.Publisher(
                        "%s/command" % (ARM_EFFORT_CONTROLLER),
                        Float64MultiArray,
                        queue_size=10,
                    )

        # Connect to gripper control services.
        if self.load_gripper and (
            control_type_group == "trajectory"
            or (control_type_group != "trajectory" and not self._direct_control)
        ):
            # Connect to 'panda_gazebo' gripper control service.
            try:
                set_gripper_width_topic = (
                    f"{self.robot_name_space}/{SET_GRIPPER_WIDTH_TOPIC}"
                )
                rospy.logdebug("Connecting to '%s' service." % set_gripper_width_topic)
                rospy.wait_for_service(
                    set_gripper_width_topic,
                    timeout=self._connection_timeout,
                )
                self._set_gripper_width_client = rospy.ServiceProxy(
                    set_gripper_width_topic, self.panda_gazebo.srv.SetGripperWidth
                )
                rospy.logdebug("Connected to '%s' service!" % set_gripper_width_topic)
                self._set_gripper_width_client_connected = True
            except (
                rospy.ServiceException,
                ROSException,
                ROSInterruptException,
            ):
                err_msg = (
                    f"Shutting down '{rospy.get_name()}' since no connection could be "
                    f"established with the '{set_gripper_width_topic}' service. This "
                    "service is needed for controlling the hand in 'PROXY' control "
                    "mode."
                )
                ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)
        elif (
            self.load_gripper
            and control_type_group != "trajectory"
            and self._direct_control
        ):
            # Connect to 'franka_gazebo' gripper command action server.
            rospy.logdebug(
                "Connecting to '%s' action service." % FRANKA_GRIPPER_COMMAND_TOPIC
            )
            franka_gripper_action_connected = True
            if action_server_exists(FRANKA_GRIPPER_COMMAND_TOPIC):
                # Connect to robot control action server.
                self._gripper_command_client = actionlib.SimpleActionClient(
                    FRANKA_GRIPPER_COMMAND_TOPIC,
                    GripperCommandAction,
                )

                # Waits until the action server has started up.
                retval = self._gripper_command_client.wait_for_server(
                    timeout=rospy.Duration(secs=5)
                )
                if not retval:
                    franka_gripper_action_connected = False
            else:
                franka_gripper_action_connected = False

            # Shutdown if franka_gripper_action not found.
            if not franka_gripper_action_connected:
                err_msg = (
                    f"Shutting down '{rospy.get_name()}' since no connection could be "
                    f"established with the '{FRANKA_GRIPPER_COMMAND_TOPIC}' action "
                    "service. This service is needed for controlling the hand in "
                    "'DIRECT' control mode."
                )
                ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)

        # Environment initiation complete message.
        rospy.logwarn("PandaEnv robot environment initialized.")

    ################################################
    # Panda Robot env main methods #################
    ################################################
    def get_ee_pose(self):
        """Returns the current end-effector pose while taking the ``ee_frame_offset``
        into account. If the offset is zero then it is equal to the
        :attr:`~PandaEnv.robot_EE_link` pose.

        Returns:
            :obj:`geometry_msgs.msg.PoseStamped`: The end-effector pose.

        Raises:
            :obj:`ros_gazebo_gym.errors.EePoseLookupError`: Error thrown when error
                occurred while trying to retrieve the EE pose.
        """
        # Retrieve EE pose, take offset into account if set.
        try:
            ee_pose = self._get_frame_pose(
                parent_frame="world",
                child_frame=self.robot_EE_link,
                offset=self.ee_frame_offset,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            logwarn_msg = "End effector pose could not be retrieved as {}".format(
                lower_first_char(e.args[0])
            )
            error_message = "End effector pose could not be retrieved."
            raise EePoseLookupError(message=error_message, log_message=logwarn_msg)

        return ee_pose

    def get_ee_rpy(self):
        """Returns the end effector EE orientation.

        Returns:
            :obj:`panda_gazebo.srv.GetEeRpyResponse`: Object containing the roll (x),
                yaw (z), pitch (y) euler angles.

        Raises:
            :obj:`ros_gazebo_gym.errors.EeRpyLookupError`: Error thrown when error
                occurred while trying to retrieve the EE rpy rotation using the
                ``get_ee_pose`` service.
        """
        try:
            # Retrieve the pose of the 'EE_frame' in the 'world' frame.
            ee_frame_pose = self._get_frame_pose(
                parent_frame="world",
                child_frame="EE_frame",
            )

            # Calculate 'EE_frame' euler angles.
            gripper_rpy = get_orientation_euler(ee_frame_pose.pose)  # Yaw, Pitch Roll.
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            logwarn_msg = (
                "End effector orientation (rpy) could not be retrieved as "
                + lower_first_char(e.args[0])
            )
            raise EeRpyLookupError(
                message="End effector orientation (rpy) could not be retrieved.",
                log_message=logwarn_msg,
            )

        return gripper_rpy

    def get_ee_pose_joint_config(self, ee_pose):
        """Returns a set of possible arm joint configurations for a given end-effector
        pose.

        Args:
            ee_pose (union[:obj:`geometry_msgs.msg.Pose`, list]): A list or pose message
                containing the end effector position (x, y, z) and orientation
                (x, y, z, w).

        Returns:
            obj:`dict`: Dictionary with joint positions that result in a given EE pose.
                Empty dictionary is returned if no joint positions could be found.
        """  # TODO ADD EE_OFFSET!
        if self._moveit_get_ee_pose_joint_config_client_connected:
            ee_pose_target = PoseStamped(
                header=Header(frame_id="world", stamp=rospy.Time(0))
            )
            ee_pose_target.pose.position.x = ee_pose["x"]
            ee_pose_target.pose.position.y = ee_pose["y"]
            ee_pose_target.pose.position.z = ee_pose["z"]
            ee_pose_target.pose.orientation.x = ee_pose["rx"]
            ee_pose_target.pose.orientation.y = ee_pose["ry"]
            ee_pose_target.pose.orientation.z = ee_pose["rz"]
            ee_pose_target.pose.orientation.w = ee_pose["rw"]
            ee_pose_target.pose.orientation = normalize_quaternion(
                ee_pose_target.pose.orientation
            )  # Make sure the orientation is normalized.

            # Remove the end-effector offset if set.
            if not self.ee_offset_is_zero:
                # FIXME: Is not yet correctly implemented.
                ee_pose_target = self._remove_ee_offset(ee_pose_target)

            # Request and return pose.
            req = self.panda_gazebo.srv.GetEePoseJointConfigRequest()
            req.pose = ee_pose_target.pose
            req.attempts = (
                self._pose_sampling_attempts
                if hasattr(self, "_pose_sampling_attempts")
                else 10
            )
            resp = self._moveit_get_ee_pose_joint_config_client.call(req)
            joint_configuration_dict = dict(zip(resp.joint_names, resp.joint_positions))
            if not resp.success:
                logdebug_msg = (
                    "Joint configuration not retrieved as "
                    + lower_first_char(resp.message)
                )
                rospy.logwarn(logdebug_msg)
                return {}
            return joint_configuration_dict

        rospy.logwarn_once(
            "Joint configuration can not be retrieved as the "
            f"{self.robot_name_space}/{MOVEIT_GET_EE_POSE_JOINT_CONFIG_TOPIC} is "
            "not available."
        )
        return {}

    def set_ee_pose(self, ee_pose):  # noqa: C901
        """Sets the Panda end effector pose.

        Args:
            ee_pose (union[:obj:`geometry_msgs.msg.Pose`, list]): A list or pose message
                containing the end effector position (x, y, z) and orientation
                (x, y, z, w).

        Returns:
            bool: Boolean specifying if the ee pose was set successfully.
        """
        if not self._moveit_set_ee_pose_client_connected:
            rospy.logwarn(
                "Setting end effector pose failed since the required service "
                f"'{MOVEIT_SET_EE_POSE_TOPIC}' was not available."
            )
            return False

        # Retrieve action space joints.
        arm_action_space_joints = [
            item
            for item in self._action_space_joints
            if item not in ["gripper_width", "gripper_max_effort"]
        ]

        # Convert scalar, list, array or tuple to joint dict.
        if isinstance(ee_pose, (list, np.ndarray, tuple)) or np.isscalar(ee_pose):
            if np.isscalar(ee_pose):
                ee_pose = [ee_pose]
            if len(ee_pose) > len(arm_action_space_joints):
                rospy.logwarn_once(
                    f"End effector pose setpoint contains {len(ee_pose)} values "
                    f"while it can only contain {len(arm_action_space_joints)}. "
                    f"As a result only the first {len(ee_pose)} "
                    "values are used in the arm and gripper control command."
                )
            ee_pose = dict(zip(arm_action_space_joints, ee_pose))

        # Create set EE pose request message.
        ee_pose_target = PoseStamped(
            header=Header(frame_id="world", stamp=rospy.Time(0))
        )
        if isinstance(ee_pose, dict):
            # Fill missing EE pose attributes with current pose values.
            if arm_action_space_joints != list(ee_pose.keys()):
                cur_ee_pose = self.get_ee_pose()
                cur_ee_pose_dict = {
                    "x": cur_ee_pose.pose.position.x,
                    "y": cur_ee_pose.pose.position.y,
                    "z": cur_ee_pose.pose.position.z,
                    "rx": cur_ee_pose.pose.orientation.x,
                    "ry": cur_ee_pose.pose.orientation.y,
                    "rz": cur_ee_pose.pose.orientation.z,
                    "rw": cur_ee_pose.pose.orientation.w,
                }
                cur_ee_pose_dict.update(ee_pose)
                ee_pose = cur_ee_pose_dict

            # Populate the ee_pose_target message.
            ee_pose_target.pose.position.x = ee_pose["x"]
            ee_pose_target.pose.position.y = ee_pose["y"]
            ee_pose_target.pose.position.z = ee_pose["z"]
            ee_pose_target.pose.orientation.x = ee_pose["rx"]
            ee_pose_target.pose.orientation.y = ee_pose["ry"]
            ee_pose_target.pose.orientation.z = ee_pose["rz"]
            ee_pose_target.pose.orientation.w = ee_pose["rw"]
        elif isinstance(ee_pose, PoseStamped):
            ee_pose_target = ee_pose
        elif isinstance(ee_pose, Pose):
            ee_pose_target.pose = ee_pose
        elif isinstance(ee_pose, self.panda_gazebo.srv.SetEePoseRequest):
            ee_pose_target.pose = ee_pose.pose
        else:  # If the ee_pose format is not valid.
            rospy.logwarn(
                "Setting end effector pose failed since the ee_pose you specified "
                f"was given as a '{type(ee_pose)}' while the 'set_ee_pose' "
                "function only accepts a list, Pose and a PoseStamped."
            )
            return False
        ee_pose_target.pose.orientation = normalize_quaternion(
            ee_pose_target.pose.orientation
        )

        # Remove the end-effector offset if set.
        if not self.ee_offset_is_zero:
            # FIXME: Is not yet correctly implemented.
            ee_pose_target = self._remove_ee_offset(ee_pose_target)

        ########################################
        # Set EE pose ##########################
        ########################################
        self._step_debug_logger(
            "Setting end effector pose using the "
            f"'{self._moveit_set_ee_pose_client.resolved_name}' service."
        )
        set_ee_pose_req = self.panda_gazebo.srv.SetEePoseRequest(
            pose=ee_pose_target.pose,
        )
        retval = self._moveit_set_ee_pose_client.call(set_ee_pose_req)
        if not retval.success:
            logdebug_msg = lower_first_char(retval.message)
            rospy.logwarn(logdebug_msg)
            return False

        return True

    def set_joint_commands(self, joint_commands, arm_wait=False, hand_wait=False):
        """Sets the Panda arm and hand joint commands based on the set
        :obj:`~PandaEnv.robot_control_type`.

        Args:
            joint_commands (union[:obj:`panda_gazebo.srv.SetJointCommands`, list, dict]):
                The Panda arm joint positions and gripper width.
            arm_wait (bool, optional): Wait till the arm control has finished. Defaults
                to ``False``.
            hand_wait (bool, optional): Wait till the hand control has finished.
                Defaults to ``False``.

        Returns:
            bool: Boolean specifying if the joint commands were set successfully.
        """  # noqa: E501
        # Set control.
        if self.robot_control_type == "effort":
            return self.set_joint_efforts(
                joint_commands,
                arm_wait=arm_wait,
                hand_wait=hand_wait,
                direct_control=self._direct_control,
            )
        else:
            return self.set_joint_positions(
                joint_commands,
                arm_wait=arm_wait,
                hand_wait=hand_wait,
                direct_control=self._direct_control,
            )

    def set_joint_positions(  # noqa: C901
        self, joint_commands, arm_wait=False, hand_wait=False, direct_control=True
    ):
        """Sets the Panda arm positions and gripper width.

        Args:
            joint_commands (union[:obj:`panda_gazebo.srv.SetJointCommands`, list, dict]):
                The Panda arm joint positions and gripper width.
            arm_wait (bool, optional): Wait till the arm control has finished. Defaults
                to ``False``.
            hand_wait (bool, optional): Wait till the hand control has finished.
                Defaults to ``False``.
            direct_control (bool): Whether we want to directly control the panda by
                publishing to the controller topics or we want to use the
                ``panda_gazebo`` control proxy. Defaults to ``True``.

        Returns:
            bool: Boolean specifying if the joint positions were set successfully.
        """  # noqa: E501
        # Convert float and array joint_commands to dictionary.
        if isinstance(joint_commands, (list, np.ndarray, tuple)) or np.isscalar(
            joint_commands
        ):
            if np.isscalar(joint_commands):
                joint_commands = [joint_commands]

            # Create joint_commands dictionary.
            if len(joint_commands) > len(self._action_space_joints):
                rospy.logwarn_once(
                    f"Joint positions setpoint contains {len(joint_commands)} values "
                    f"while it can only contain {len(self._action_space_joints)}. "
                    f"As a result only the first {len(joint_commands)} "
                    "values are used in the arm and gripper control command."
                )
            joint_commands = dict(zip(self._action_space_joints, joint_commands))

        # Throw warning and return if joint_commands is not of the correct type.
        valid_types = (
            (dict, list, np.ndarray, tuple)
            if direct_control
            else (dict, self.panda_gazebo.srv.SetJointCommandsRequest)
        )
        if not isinstance(joint_commands, valid_types):
            control_mode = "DIRECT" if direct_control else "INDIRECT"
            valid_types_str = list_2_human_text(
                [f"'{type_.__name__}'" for type_ in valid_types],
                separator=", a",
                end_separator=" or a",
            )
            rospy.logwarn(
                "Setting joint positions failed since the 'joint_commands' argument is "
                f"of the '{type(joint_commands).__name__}' type while the "
                f"'set_joint_positions' function only accepts a {valid_types_str} when "
                f"{control_mode} control mode."
            )
            return False

        ########################################
        # Set joint positions ##################
        ########################################
        commanded_joints = (
            list(joint_commands.keys())
            if isinstance(joint_commands, dict)
            else joint_commands.joint_names
        )
        if (
            "gripper_max_effort" in commanded_joints
            and "gripper_width" not in commanded_joints
        ):
            rospy.logwarn_once(
                "Gripper max effort was specified but no gripper width was specified. "
                "As a result the max effort will be ignored."
            )
        self._step_debug_logger(
            "Setting joint positions using {} control mode.".format(
                "DIRECT" if direct_control else "PROXY"
            )
        )
        if not direct_control:  # Use proxy service.
            # Create SetJointCommandsRequest message.
            if isinstance(joint_commands, dict):
                req = self.panda_gazebo.srv.SetJointCommandsRequest()
                req.grasping = self._grasping if self._grasping else False
                req.joint_names = list(joint_commands.keys())
                req.joint_commands = list(joint_commands.values())
                req.control_type = "position"
            elif isinstance(
                joint_commands, self.panda_gazebo.srv.SetJointCommandsRequest
            ):
                req = joint_commands
            req.arm_wait = arm_wait
            req.hand_wait = hand_wait

            # Ensure that the gripper is blocked if self.lock_gripper is True.
            if self.lock_gripper:
                req = self._block_gripper_commands(req)

            # Prevent duplicate gripper commands from being sent.
            if is_sublist(["gripper_width", "gripper_max_effort"], req.joint_names):
                if self._last_gripper_goal != req:
                    self._last_gripper_goal = req
                else:
                    req = remove_gripper_commands_from_joint_commands_msg(req)

            # Send arm and hand control commands.
            if self._set_joint_commands_client_connected:
                self._set_joint_commands_client.call(req)
            else:
                rospy.logwarn(
                    "Setting joints positions failed since the "
                    f"'{SET_JOINT_COMMANDS_TOPIC}' service was not available."
                )
                return False
        else:  # Directly control the controllers.
            self._joint_positions_direct_control(
                joint_commands, arm_wait=arm_wait, hand_wait=hand_wait
            )
        return True

    def _joint_positions_direct_control(
        self, joint_commands, arm_wait=False, hand_wait=False
    ):
        """Directly publish position commands to the controller command topics. This
        is faster but does not wait for the control command to complete.

        Args:
            joint_commands (dict): The panda joint positions and gripper width.
            arm_wait (bool, optional): Wait till the arm control has finished. Defaults
                to ``False``.
            hand_wait (bool, optional): Wait till the hand control has finished.
                Defaults to ``False``.
        """
        # Retrieve arm and gripper commands.
        arm_commands = self._get_arm_commands(
            joint_commands, control_type="position", fill_missing=True
        )
        gripper_width, gripper_max_effort = self._get_gripper_commands(joint_commands)

        # Send arm and hand control commands.
        self._arm_joint_position_pub.publish(
            Float64MultiArray(data=list(arm_commands.values()))
        )
        if arm_wait:
            self._wait_till_arm_control_done(
                control_type="position",
                joint_setpoint=list(arm_commands.values()),
            )
        if self.load_gripper and not self.lock_gripper and gripper_width is not None:
            req = GripperCommandGoal()
            req.command.position = (
                gripper_width / 2
            )  # NOTE: Done the action expects the finger width.
            req.command.max_effort = gripper_max_effort

            # Send gripper goal if it was different from the previous one.
            if self._last_gripper_goal != req:
                self._last_gripper_goal = req
                self._gripper_command_client.send_goal(req)
                if hand_wait:
                    self._gripper_command_client.wait_for_result(
                        timeout=rospy.Duration(secs=5)
                    )

    def set_joint_efforts(  # noqa: C901
        self, joint_commands, arm_wait=False, hand_wait=False, direct_control=True
    ):
        """Sets the Panda arm efforts and the gripper width.

        Args:
            joint_commands (union[:obj:`panda_gazebo.srv.SetJointCommandsRequest`, list, dict]):
                The panda joint efforts and gripper width.
            direct_control (bool): Whether we want to directly control the panda by
                publishing to the controller topics or we want to use the
                ``panda_gazebo`` control proxy. Defaults to ``True``.
            arm_wait (bool, optional): Wait till the arm control has finished. Defaults
                to ``False``.
            hand_wait (bool, optional): Wait till the hand control has finished.
                Defaults to ``False``.

        Returns:
            bool: Boolean specifying if the joint efforts were set successfully.
        """  # noqa: E501
        # Convert float and array joint_commands to dictionary.
        if isinstance(joint_commands, (list, np.ndarray, tuple)) or np.isscalar(
            joint_commands
        ):
            if np.isscalar(joint_commands):
                joint_commands = [joint_commands]

            # Create joint_commands dictionary.
            if len(joint_commands) > len(self._action_space_joints):
                rospy.logwarn_once(
                    f"Joint efforts setpoint contains {len(joint_commands)} values "
                    f"while it can only contain {len(self._action_space_joints)}. "
                    f"As a result only the first {len(joint_commands)} "
                    "values are used in the arm and gripper control command."
                )
            joint_commands = dict(zip(self._action_space_joints, joint_commands))

        # Throw warning and return if joint_commands is not of the correct type.
        valid_types = (
            (dict, list, np.ndarray, tuple)
            if direct_control
            else (dict, self.panda_gazebo.srv.SetJointCommandsRequest)
        )
        if not isinstance(joint_commands, valid_types):
            control_mode = "DIRECT" if direct_control else "INDIRECT"
            valid_types_str = list_2_human_text(
                [f"'{type_.__name__}'" for type_ in valid_types],
                separator=", a",
                end_separator=" or a",
            )
            rospy.logwarn(
                "Setting joint efforts failed since the 'joint_commands' argument is "
                f"of the '{type(joint_commands).__name__}' type while the "
                f"'set_joint_efforts' function only accepts a {valid_types_str} when "
                f"{control_mode} control mode."
            )
            return False

        ########################################
        # Set joint efforts ####################
        ########################################
        commanded_joints = (
            list(joint_commands.keys())
            if isinstance(joint_commands, dict)
            else joint_commands.joint_names
        )
        if (
            "gripper_max_effort" in commanded_joints
            and "gripper_width" not in commanded_joints
        ):
            rospy.logwarn_once(
                "Gripper max effort was specified but no gripper width was specified. "
                "As a result the max effort will be ignored."
            )
        self._step_debug_logger(
            "Setting joint efforts using {} control mode.".format(
                "DIRECT" if direct_control else "PROXY"
            )
        )
        if not direct_control:  # Use proxy service.
            # Create SetJointCommandsRequest message.
            if isinstance(joint_commands, dict):
                req = self.panda_gazebo.srv.SetJointCommandsRequest()
                req.grasping = self._grasping if self._grasping else False
                req.joint_names = list(joint_commands.keys())
                req.joint_commands = list(joint_commands.values())
                req.control_type = "effort"
            elif isinstance(
                joint_commands, self.panda_gazebo.srv.SetJointCommandsRequest
            ):
                req = joint_commands
            req.arm_wait = arm_wait
            req.hand_wait = hand_wait

            # Ensure that the gripper is blocked if self.lock_gripper is True.
            if self.lock_gripper:
                req = self._block_gripper_commands(req)

            # Send arm and hand control commands.
            if self._set_joint_commands_client_connected:
                self._set_joint_commands_client.call(req)
            else:
                rospy.logwarn(
                    "Setting joints efforts failed since the "
                    f"'{SET_JOINT_COMMANDS_TOPIC}' service was not available."
                )
                return False
        else:  # Directly control the controllers.
            self._joint_efforts_direct_control(
                joint_commands, arm_wait=arm_wait, hand_wait=hand_wait
            )

        return True

    def _joint_efforts_direct_control(
        self,
        joint_commands,
        arm_wait=False,  # Currently not used.
        hand_wait=False,
    ):
        """Directly publish effort commands to the controller command topics. This is
        faster but does not wait for the control command to complete.

        Args:
            joint_commands (dict): The panda joint efforts and gripper width.
            hand_wait (bool, optional): Wait till the hand control has finished.
                Defaults to ``False``.
        """
        # Retrieve arm and gripper commands.
        arm_commands = self._get_arm_commands(
            joint_commands, control_type="effort", fill_missing=True
        )
        gripper_width, gripper_max_effort = self._get_gripper_commands(joint_commands)

        # Send arm and hand control commands.
        self._arm_joint_effort_pub.publish(
            Float64MultiArray(data=list(arm_commands.values()))
        )
        # NOTE: We currently do not have to wait for control efforts to be applied
        # since the 'FrankaHWSim' does not yet implement control latency. Torques
        # are therefore applied instantly.
        # if arm_wait:
        #     self._wait_till_arm_control_done(
        #         control_type="effort",
        #         joint_setpoint=list(arm_commands.values())
        #     )
        if self.load_gripper and not self.lock_gripper and gripper_width is not None:
            req = GripperCommandGoal()
            req.command.position = (
                gripper_width / 2
            )  # NOTE: Done the action expects the finger width.
            req.command.max_effort = gripper_max_effort
            self._gripper_command_client.send_goal(req)
            if hand_wait:
                self._gripper_command_client.wait_for_result(
                    timeout=rospy.Duration(secs=5)
                )

    def set_arm_joint_trajectory(  # noqa: C901
        self, joint_trajectory, wait=False, time_from_start=None
    ):
        """Sets the panda arm joint trajectory.

        Args:
            joint_trajectory (union[:obj:`panda_gazebo.msg.FollowJointTrajectoryActionGoal`, np.array, list, dict])
                The joint trajectory you want to set.
            wait  (bool, optional): Wait till the control has finished. Defaults to
                ``False``.
            time_from_start (float): At what time a trajectory point
                should be reach in seconds. Only used when a single trajectory point is
                given (i.e. list, dict)

        Returns:
            bool: Boolean specifying if the joint trajectory was set successfully.

        Raises:
            ValueError: If the input trajectory is invalid.

        .. note::
            If you input a ``tuple``, ``list``, ``int`` or ``float`` a joint trajectory
            message will be constructed with one waypoint point. To set multiple
            waypoints, you must supply a 2D numpy array or a dictionary containing an
            equal number of joint commands for each joint, one for each waypoint.
        """  # noqa: E501
        if self._arm_joint_traj_control_client_connected:
            arm_action_space_joints = [
                item
                for item in self._action_space_joints
                if item not in ["gripper_width", "gripper_max_effort"]
            ]
            # Convert float and array joint_trajectory to dictionary.
            if (
                isinstance(joint_trajectory, (list, tuple))
                or np.isscalar(joint_trajectory)
                or (
                    isinstance(joint_trajectory, (np.ndarray))
                    and joint_trajectory.ndim == 1
                )
            ):
                if np.isscalar(joint_trajectory):
                    joint_trajectory = [joint_trajectory]

                # Create joint trajectory dictionary.
                if len(joint_trajectory) > len(arm_action_space_joints):
                    rospy.logwarn_once(
                        f"Joint trajectory contains {len(joint_trajectory)} values "
                        f"while it can only contain {len(arm_action_space_joints)}. "
                        f"As a result only the first {len(joint_trajectory)} "
                        "values are used in the arm and gripper control command."
                    )
                joint_trajectory = dict(zip(arm_action_space_joints, joint_trajectory))
            elif (
                isinstance(joint_trajectory, (np.ndarray))
                and joint_trajectory.ndim == 2
            ):
                if joint_trajectory.shape[1] > len(arm_action_space_joints):
                    rospy.logwarn_once(
                        f"Joint trajectory contains {len(joint_trajectory)} values "
                        f"while it can only contain {len(arm_action_space_joints)}. "
                        f"As a result only the first {len(joint_trajectory)} "
                        "values are used in the arm and gripper control command."
                    )
                joint_trajectory = dict(
                    zip(arm_action_space_joints, np.transpose(joint_trajectory))
                )
            elif (
                isinstance(joint_trajectory, (np.ndarray)) and joint_trajectory.ndim > 2
            ):
                raise ValueError(
                    "The joint trajectory could not be set since the "
                    "'joint_trajectory' you supplied is invalid. If you want to send "
                    "multiple waypoints the 'joint_trajectory' should be a 2D array or "
                    "a dictionary containing an equal number of joint commands for "
                    "each joint, one for each waypoint."
                )

            # Create SetJointtrajectory message.
            if isinstance(joint_trajectory, dict):
                time_from_start = time_from_start if time_from_start else 0.01
                time_axis_step = (
                    time_from_start / max(list(joint_trajectory.values())[0].shape)
                    if isinstance(list(joint_trajectory.values())[0], np.ndarray)
                    else time_from_start
                )
                req = (
                    self.panda_gazebo.common.helpers.action_dict_2_joint_trajectory_msg(
                        joint_trajectory, time_axis_step=time_axis_step
                    )
                )
            elif isinstance(
                joint_trajectory, self.panda_gazebo.msg.FollowJointTrajectoryGoal
            ):
                req = joint_trajectory
            else:
                rospy.logwarn(
                    "Setting joint trajectory failed since the 'joint_trajectory' "
                    f"input argument is of the '{type(joint_trajectory)}' type while "
                    "the 'set_arm_joint_trajectory' function only accepts a dictionary, "
                    "list or a FollowJointTrajectoryActionGoal message."
                )
                return False

            ########################################
            # Set trajectory #######################
            ########################################
            # Try to setting joint trajectory if service is available.
            self._step_debug_logger(
                "Setting joint trajectory using the "
                f"'{self._arm_joint_traj_control_client.action_client.ns}' action "
                "service."
            )
            self._arm_joint_traj_control_client.send_goal(req)
            retval = True
            if wait:
                retval = self._arm_joint_traj_control_client.wait_for_result()
            if not retval:
                rospy.logwarn("Setting joint trajectory failed.")
                return False

            return True

        rospy.logwarn(
            "Setting joints trajectory failed since the "
            f"'{SET_JOINT_TRAJECTORY_TOPIC}' service was not available."
        )
        return False

    def set_gripper_width(
        self, gripper_width, wait=None, grasping=None, max_effort=None
    ):
        """Sets the Panda gripper width.

        Args:
            gripper_width (float): The desired gripper width.
            wait (bool, optional): Wait till the gripper control has finished. Defaults
                to ``False``.
            grasp (bool): Whether we want to grasp a object. Defaults to ``False``. Can
                be overwritten by setting the ``<NS>/control/grasping`` ROS parameter.
            max_effort (float, optional): The max effort used when grasping. Defaults to
                ``None``. Meaning the action service default is used.

        Returns:
            bool: Boolean specifying if the gripper width was set successfully.
        """
        if self._set_gripper_width_client_connected:
            grasping = grasping if grasping is not None else self._grasping
            self._step_debug_logger(
                "Setting gripper width using the "
                f"'{self._set_gripper_width_client.resolved_name}' service."
            )

            # Create gripper width request.
            req = self.panda_gazebo.srv.SetGripperWidthRequest()
            req.width = gripper_width
            if grasping is not None:
                req.grasping = grasping
            if max_effort is not None:
                req.max_effort = max_effort
            if wait:  # Done since it will otherwise use the msg default value.
                req.wait = wait

            # Send gripper width request.
            self._set_gripper_width_client.call(req)
            return True

        rospy.logwarn(
            f"Setting gripper_width failed since the '{SET_JOINT_COMMANDS_TOPIC}' "
            "service was not available."
        )
        return False

    def _wait_till_arm_control_done(  # noqa: C901
        self,
        control_type,
        joint_setpoint,
        timeout=None,
        check_gradient=True,
    ):
        """Wait till arm control is finished. Meaning the robot state is within range
        of the joint position and joint effort setpoints (or the velocity is zero).

        Args:
            control_type (str): The type of control that is being executed and on which
                we should wait. Options are ``effort`` and ``position``.
            joint_setpoint (list): The setpoint to wait for.
            timeout (int, optional): The timeout when waiting for the control
                to be done. Defaults to
                :attr:`~PandaControlServer._wait_till_arm_control_done_timeout`.
            check_gradient (boolean, optional): If enabled the script will also return
                when the gradients become zero. Defaults to ``True``.

        Raises:
            :obj:`ValueError`: Raised when the control_type is invalid.
        """
        if control_type not in ["position", "effort"]:
            raise ValueError(
                "Please specify a valid control type. Valid values are 'position' & "
                "'effort'."
            )
        else:
            control_type = control_type.lower()
        timeout = (
            rospy.Duration(timeout)
            if timeout is not None
            else rospy.Duration(ARM_CONTROL_WAIT_TIMEOUT)
        )

        # Compute the state masks.
        try:
            arm_states_mask = [
                joint in self.joints["arm"] for joint in self.joint_states.name
            ]
        except self.panda_gazebo.exceptions:
            rospy.logwarn(
                "Not waiting for control to be completed as no information could "
                "be retrieved about which joints are controlled when using '%s' "
                "control. Please make sure the '%s' controller that is needed for '%s' "
                "control are initialized."
                % (
                    control_type,
                    (
                        ARM_POSITION_CONTROLLER
                        if control_type == "position"
                        else ARM_EFFORT_CONTROLLER
                    ),
                    control_type,
                )
            )
            return False
        if not any(arm_states_mask):
            rospy.logwarn(
                "Not waiting for control to be completed as no joints appear to be "
                "controlled when using '%s' control. Please make sure the '%s' "
                "controller that is needed for '%s' control are initialized."
                % (
                    control_type,
                    (
                        ARM_POSITION_CONTROLLER
                        if control_type == "position"
                        else ARM_EFFORT_CONTROLLER
                    ),
                    control_type,
                )
            )
            return False

        # Wait till robot positions/efforts reach the setpoint or the velocities are
        # not changing anymore.
        timeout_time = rospy.get_rostime() + timeout
        while not rospy.is_shutdown() and rospy.get_rostime() < timeout_time:
            # Wait till joint positions/efforts are within range or vel not changing.
            joint_states = (
                np.array(list(compress(self.joint_states.position, arm_states_mask)))
                if control_type == "position"
                else np.array(list(compress(self.joint_states.effort, arm_states_mask)))
            )
            state_threshold = (
                ARM_JOINT_POSITION_WAIT_THRESHOLD
                if control_type == "position"
                else ARM_JOINT_EFFORT_WAIT_THRESHOLD
            )
            grad_threshold = ARM_JOINT_VELOCITY_WAIT_THRESHOLD
            joint_setpoint_tmp = np.append(
                np.array(joint_setpoint),
                joint_states[len(joint_setpoint) :],  # noqa: E203, E501
            )

            # Add current state to state_buffer and delete oldest entry.
            state_buffer = np.full((2, len(joint_states)), np.nan)
            grad_buffer = np.full((2, len(joint_states)), np.nan)
            state_buffer = np.delete(
                np.append(state_buffer, [joint_states], axis=0), 0, axis=0
            )
            grad_buffer = np.gradient(state_buffer, axis=0)

            # Check if setpoint is reached.
            if check_gradient:  # Check position/effort and gradients.
                if (
                    np.linalg.norm(joint_states - joint_setpoint_tmp)
                    <= state_threshold  # Check if difference norm is within threshold.
                ) or all(
                    [
                        (np.abs(val) <= grad_threshold and val != 0.0)
                        for val in grad_buffer[-1]
                    ]  # Check if all velocities are close to zero.
                ):
                    break
            else:  # Only check position/effort.
                if (
                    np.linalg.norm(joint_states - joint_setpoint_tmp)
                    <= state_threshold  # Check if difference norm is within threshold.
                ):
                    break

        return True

    def lock_joints(self, joint_names):
        """Locks specific panda joints.

        Args:
            joint_names (list): The names of the joints to lock.
        """
        resp = self._joint_lock_client.call(
            self.panda_gazebo.srv.LockJointsRequest(joint_names=joint_names, lock=True)
        )
        self.__locked_joints = joint_names if resp.success else []

    def unlock_joints(self, joint_names):
        """Unlocks specific panda joints.

        Args:
            joint_names (list): The names of the joints to unlock.
        """
        resp = self._joint_lock_client.call(
            self.panda_gazebo.srv.LockJointsRequest(joint_names=joint_names, lock=False)
        )
        self.__locked_joints = [] if resp.success else joint_names

    ################################################
    # Panda Robot env helper methods ###############
    ################################################
    def _joint_states_cb(self, data):
        """Joint states subscriber callback function.

        Args:
            data (:obj:`sensor_msgs.msg.JointState`): The data that is returned by the
                subscriber.
        """
        self.joint_states = data

    def _franka_states_cb(self, data):
        """Franka states subscriber callback function.

        Args:
            data (:obj:`franka_msgs.msg.FrankaState`): The data that is returned by the
                subscriber.
        """
        self.franka_states = data
        self.__in_collision = any(self.franka_states.cartesian_contact)
        if not (rospy.is_shutdown() or self._ros_shutdown_requested):
            self._in_collision_pub.publish(
                Float32(np.float32(self._PandaEnv__in_collision))
            )

    def _check_all_sensors_ready(self):
        """Checks whether we are receiving sensor data."""
        self._check_joint_states_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        """Checks if we are receiving joint state sensor data.

        Returns:
            :obj:`sensor_msgs.msgs.JointState`: Array containing the joint states.
        """
        self.joint_states = None
        while self.joint_states is None and not rospy.is_shutdown():
            try:
                self.joint_states = rospy.wait_for_message(
                    JOINT_STATES_TOPIC, JointState, timeout=self._connection_timeout
                )
                rospy.logdebug(
                    f"Current '{JOINT_STATES_TOPIC}' READY=>" + str(self.joint_states)
                )
            except ROSException:
                rospy.logwarn(
                    f"Current '{JOINT_STATES_TOPIC}' not ready yet, retrying for "
                    "getting joint_states."
                )
        return self.joint_states

    def _step_debug_logger(self, *args, **kwargs):
        """Small wrapper method around the :obj:`rospy.logdebug` method that blocks
        logging when the :obj:`~PandaReachEnv._log_step_debug_info` parameter is
        ``False``.

        .. note::
            Done to increase the step frequency.
        """
        if self._log_step_debug_info:
            rospy.logdebug(*args, **kwargs)
        else:
            pass

    def _ros_shutdown_hook(self):
        """Method that is called when the ROS node is shutdown."""
        self._ros_shutdown_requested = True

    def _fill_missing_arm_joint_commands(self, joint_commands, control_type):
        """Fills missing arm joint commands with the current joint states if the joint
        states dictionary is incomplete.

        Args:
            joint_commands (dict): The joint commands dictionary.
            control_type (str): The type of control commands that are being executed.
                Options are ``effort`` and ``position``.

        Returns:
            dict: The arm joint commands dictionary with the missing joint states
                filled in.
        """
        cur_joint_commands = (
            self.arm_positions
            if control_type.lower() == "position"
            else self.arm_efforts
        )
        cur_joint_commands.update(joint_commands)
        joint_commands = cur_joint_commands
        return joint_commands

    def _get_arm_commands(self, joint_commands, control_type, fill_missing=False):
        """Retrieves the arm commands from the joint commands dictionary.

        Args:
            joint_commands (dict): The joint commands dictionary.
            control_type (str): The type of control commands that are being executed.
                Options are ``effort`` and ``position``.
            fill_missing (bool, optional): Whether to fill missing arm joint commands
                with the current joint states if the joint states dictionary is
                incomplete. Defaults to ``False``.

        Returns:
            dict: The arm joint commands dictionary.
        """
        # Remove gripper commands.
        arm_commands = copy.deepcopy(joint_commands)
        arm_commands.pop("gripper_width", None)
        arm_commands.pop("gripper_max_effort", None)

        # Fill missing arm joint commands if needed.
        if fill_missing:
            arm_commands = self._fill_missing_arm_joint_commands(
                arm_commands, control_type=control_type
            )
        return arm_commands

    def _get_gripper_commands(self, joint_commands):
        """Retrieves the gripper commands from the joint commands dictionary.

        Args:
            joint_commands (dict): The joint commands dictionary.

        Returns:
            (tuple): tuple containing:

                - **gripper_width** (float): The gripper width.
                - **gripper_max_effort** (float): The gripper max effort.
        """
        gripper_width = None
        gripper_max_effort = None
        if self.load_gripper and not self.lock_gripper:
            # Block gripper if requested and set max effort to 10N if grasping.
            gripper_width = joint_commands.pop("gripper_width", None)
            gripper_max_effort = joint_commands.pop(
                "gripper_max_effort", GRASP_FORCE if self._grasping else 0.0
            )
        return gripper_width, gripper_max_effort

    def _block_gripper_commands(self, joint_commands_msg):
        """Modifies the joint commands message to block the gripper commands.

        Args:
            joint_commands_msg (:obj:`panda_gazebo.msg.JointCommands`): The joint
                commands message.
        """
        if self.load_gripper:
            joint_commands_msg.grasping = False
            joint_commands_msg = remove_gripper_commands_from_joint_commands_msg(
                joint_commands_msg
            )
        return joint_commands_msg

    def _add_ee_frame(self):
        """Adds the end-effector frame to the TF tree.

        .. note::
            Created based on the end effector link and the offset specified in the
            configuration file.
        """
        ee_frame_offset = self.ee_frame_offset
        self._static_tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(frame_id=self.robot_EE_link, stamp=rospy.Time.now()),
                child_frame_id="EE_frame",
                transform=Transform(
                    translation=ee_frame_offset.position,
                    rotation=normalize_quaternion(ee_frame_offset.orientation),
                ),
            )
        )

    def _get_frame_pose(self, parent_frame, child_frame, offset=None, timeout=1.0):
        """Retrieves the pose of the child frame relative to the parent frame.

        Args:
            parent_frame (str): The parent frame.
            child_frame (str): The child frame.
            offset (:obj:`geometry_msgs.msg.Pose`, optional): The offset of the child
                frame relative to the parent frame. Defaults to ``None`` meaning no
                offset is applied.
            timeout (float, optional): The timeout in seconds. Defaults to 1.0.

        Returns:
            :obj:`geometry_msgs.msg.StampedPose`: The stamped pose of the child frame
                relative to the parent frame.

        Raises:
            TimeoutError: Raised when the timeout is exceeded.
        """
        offset_pose = (
            offset
            if offset is not None
            else Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))
        )

        # Retrieve transform between child and the parent frame.
        try:
            # NOTE: Used instead of 'lookup_transform' to prevent flipped Quaternion.
            return self.tf_buffer.transform(
                PoseStamped(
                    header=Header(frame_id=child_frame, stamp=rospy.Time(0)),
                    pose=offset_pose,
                ),
                parent_frame,
                rospy.Duration(timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            err_msg = (
                f"Could not retrieve (stamped) pose of the '{child_frame}' relative "
                f"to the '{parent_frame}' frame. Please check that these frames are "
                "available in the tf tree."
            )
            raise TimeoutError(err_msg)

    def _remove_ee_offset(self, ee_pose):
        """Remove end-effector offset from end-effector pose.

        Args:
            ee_pose (:obj:`geometry_msgs.msg.PoseStamped`): The end-effector pose.

        Returns:
            :obj:`geometry_msgs.msg.PoseStamped`: The end-effector pose adjusted for
                the end-effector offset.
        """
        offset_pose = self.ee_frame_offset_stamped

        # Transform ee_pose to end-effector frame.
        try:
            transformed_ee_pose = self.tf_buffer.transform(
                ee_pose, self.robot_EE_link, rospy.Duration(1.0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            err_msg = (
                "Failed to transform the end-effector pose to the end-effector frame."
            )
            raise TimeoutError(err_msg)

        # Remove position offset from ee_pose.
        # FIXME: This approach is naive and does not take into account whether the
        # orientation is flipped.
        transformed_ee_pose.pose.position.x -= offset_pose.pose.position.x
        transformed_ee_pose.pose.position.y -= offset_pose.pose.position.y
        transformed_ee_pose.pose.position.z -= offset_pose.pose.position.z

        # Remove orientation offset from ee_pose.
        inv_offset_orientation = quaternion_inverse(
            [
                offset_pose.pose.orientation.x,
                offset_pose.pose.orientation.y,
                offset_pose.pose.orientation.z,
                offset_pose.pose.orientation.w,
            ]
        )
        ee_pose_orientation = [
            transformed_ee_pose.pose.orientation.x,
            transformed_ee_pose.pose.orientation.y,
            transformed_ee_pose.pose.orientation.z,
            transformed_ee_pose.pose.orientation.w,
        ]
        ee_pose_orientation = quaternion_multiply(
            ee_pose_orientation, inv_offset_orientation
        )
        transformed_ee_pose.pose.orientation.x = ee_pose_orientation[0]
        transformed_ee_pose.pose.orientation.y = ee_pose_orientation[1]
        transformed_ee_pose.pose.orientation.z = ee_pose_orientation[2]
        transformed_ee_pose.pose.orientation.w = ee_pose_orientation[3]

        # Transform the pose back to the world frame
        try:
            final_ee_pose = self.tf_buffer.transform(
                transformed_ee_pose, "world", rospy.Duration(1.0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            err_msg = (
                "Failed to transform the offset adjusted end-effector pose back to "
                "the world frame."
            )
            rospy.logerr("Transform failed: %s", e)
            raise

        return final_ee_pose

    #############################################
    # Retrieve robot states #####################
    #############################################
    @property
    def joints(self):
        """Returns the joints that can be controlled when using the current Panda arm
        and hand control types.

        .. important::
            For performance reasons the controlled joints are currently fetched at the
            first call. Please call the :meth:`~PandaEnv.refresh_joints` method to
            re-fetch the currently controlled joints.
        """
        if not self.__joints:
            if self._get_controlled_joints_client_connected:
                resp = self._get_controlled_joints_client.call(
                    self.panda_gazebo.srv.GetControlledJointsRequest(
                        control_type=self.robot_control_type
                    )
                )
                self.__joints["arm"] = (
                    resp.controlled_joints_arm
                    if (resp.success and resp.controlled_joints_arm)
                    else PANDA_JOINTS_FALLBACK["arm"]
                )
                self.__joints["hand"] = (
                    resp.controlled_joints_hand
                    if (
                        not self.load_gripper
                        or resp.success
                        and resp.controlled_joints_hand
                    )
                    else PANDA_JOINTS_FALLBACK["hand"]
                )
                self.__joints["both"] = (
                    flatten_list([self.__joints["arm"], self.__joints["hand"]])
                    if self.joint_states.name[0] in self.__joints["arm"]
                    else flatten_list([self.__joints["hand"], self.__joints["arm"]])
                )
            else:
                rospy.logwarn_once(
                    "Retrieving controlled joints failed since the "
                    f"'{GET_CONTROLLED_JOINTS_TOPIC}' service was not available."
                )
        return self.__joints

    def refresh_joints(self):
        """Re-fetches the currently active joints."""
        self.__joints = {}

    @property
    def gripper_width(self):
        """Returns the gripper width as calculated based on the Panda finger joints.

        Returns:
            float: The gripper width.
        """
        return (
            dict(zip(self.joint_states.name, self.joint_states.position))[
                PANDA_JOINTS_FALLBACK["hand"][0]
            ]
            * 2
        )

    @property
    def arm_positions(self):
        """Returns the current arm joint positions.

        Returns:
            dict: The arm joint positions.
        """
        return {
            key: val
            for key, val in zip(self.joint_states.name, self.joint_states.position)
            if key in self.joints["arm"]
        }

    @property
    def arm_velocities(self):
        """Returns the current arm joint velocities.

        Returns:
            dict: The arm joint velocities.
        """
        return {
            key: val
            for key, val in zip(self.joint_states.name, self.joint_states.velocity)
            if key in self.joints["arm"]
        }

    @property
    def arm_efforts(self):
        """Returns the current arm joint efforts.

        Returns:
            dict: The arm joint efforts.
        """
        return {
            key: val
            for key, val in zip(self.joint_states.name, self.joint_states.effort)
            if key in self.joints["arm"]
        }

    @property
    def robot_control_type(self):
        """Returns the currently set robot control type."""
        return self.__robot_control_type

    @robot_control_type.setter
    def robot_control_type(self, control_type):
        """Sets the robot control type while making sure the required controllers are
        loaded. Options are: ``trajectory``, ``position`` and ``effort``.
        """
        # Make sure the controller are running.
        resp = self._controller_switcher.switch(
            control_group="arm", control_type=control_type, verbose=True
        )
        if resp.success:
            self.__robot_control_type = control_type
        else:
            err_msg = (
                f"Shutting down '{rospy.get_name()}' since the controllers required "
                f"for '{control_type}' control were not loaded successfully."
            )
            ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)

    @property
    def in_collision(self):
        """Whether the robot is in collision."""
        return self.__in_collision

    @property
    def locked_joints(self):
        """Returns the currently locked joints."""
        return self.__locked_joints

    @property
    def ee_link_exists(self):
        """Returns whether the end effector link exists in the robot model.

        Returns:
            bool: Whether the end effector link exists in the robot model.
        """
        return any(link.name == self.robot_EE_link for link in self._robot.links)

    @property
    def ee_pose(self):
        """Returns the current end-effector pose while taking the ``ee_frame_offset``
        into account. If the offset is zero then it is equal to the
        :attr:`~PandaEnv.robot_EE_link` pose.

        Returns:
            :obj:`geometry_msgs.msg.PoseStamped`: The end-effector pose.
        """
        return self.get_ee_pose()

    @property
    def ee_offset_is_zero(self):
        """Returns whether the end-effector frame offset is zero.

        Returns:
            bool: Whether the end-effector frame offset is zero.
        """
        return sum(self._ee_frame_offset_dict.values()) == 1.0

    @property
    def ee_frame_offset(self):
        """Returns the end-effector frame offset relative to the actual ee_link (i.e.
        :attr:`~PandaEnv.robot_EE_link`).

        Returns:
            :obj:`geometry_msgs.msg.Pose`: The ee frame offset pose.
        """
        return Pose(
            position=Vector3(
                x=self._ee_frame_offset_dict.get("x", 0.0),
                y=self._ee_frame_offset_dict.get("y", 0.0),
                z=self._ee_frame_offset_dict.get("z", 0.0),
            ),
            orientation=normalize_quaternion(
                Quaternion(
                    x=self._ee_frame_offset_dict.get("rx", 0.0),
                    y=self._ee_frame_offset_dict.get("ry", 0.0),
                    z=self._ee_frame_offset_dict.get("rz", 0.0),
                    w=self._ee_frame_offset_dict.get("rw", 0.0),
                )
            ),
        )

    @property
    def ee_frame_offset_stamped(self):
        """Returns the end-effector frame offset relative to the actual ee_link (i.e.
        :attr:`~PandaEnv.robot_EE_link`) as a stamped pose.

        Returns:
            :obj:`geometry_msgs.msg.PoseStamped`: The ee frame offset pose.
        """
        return PoseStamped(
            header=Header(frame_id=self.robot_EE_link, stamp=rospy.Time(0)),
            pose=self.ee_frame_offset,
        )

    ################################################
    # Overload Gazebo env virtual methods ##########
    ################################################
    # NOTE: Methods needed by the gazebo environment.
    def _check_all_systems_ready(self):
        """Checks that all the sensors, publishers and other simulation systems are
        operational.

        Returns:
            bool: Boolean specifying whether reset was successful.
        """
        self._check_all_sensors_ready()
        return True
