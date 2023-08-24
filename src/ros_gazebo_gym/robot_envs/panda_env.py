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
import sys
from datetime import datetime
from itertools import compress

import actionlib
import numpy as np
import rospy
import tf2_ros
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from geometry_msgs.msg import Pose, PoseStamped
from rospy.exceptions import ROSException, ROSInterruptException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float32

from ros_gazebo_gym.common.helpers import (
    action_server_exists,
    flatten_list,
    get_orientation_euler,
    lower_first_char,
    normalize_quaternion,
)
from ros_gazebo_gym.core import ROSLauncher, LazyImporter
from ros_gazebo_gym.core.helpers import get_log_path
from ros_gazebo_gym.exceptions import EePoseLookupError, EeRpyLookupError
from ros_gazebo_gym.robot_gazebo_goal_env import RobotGazeboGoalEnv


# Specify topics and connection timeouts.
CONNECTION_TIMEOUT = 5  # Timeout for connecting to services or topics.
GAZEBO_SIM_CONNECTION_TIMEOUT = 60  # Timeout for waiting for gazebo to be launched.
MOVEIT_SET_EE_POSE_TOPIC = "panda_moveit_planner_server/panda_arm/set_ee_pose"
MOVEIT_GET_EE_POSE_JOINT_CONFIG_TOPIC = (
    "panda_moveit_planner_server/panda_arm/get_ee_pose_joint_config"
)
GET_CONTROLLED_JOINTS_TOPIC = "panda_control_server/get_controlled_joints"
SET_JOINT_COMMANDS_TOPIC = "panda_control_server/set_joint_commands"
SET_GRIPPER_WIDTH_TOPIC = "panda_control_server/panda_hand/set_gripper_width"
SET_JOINT_TRAJECTORY_TOPIC = "panda_control_server/panda_arm/follow_joint_trajectory"
FRANKA_GRIPPER_COMMAND_TOPIC = "franka_gripper/gripper_action"
JOINT_STATES_TOPIC = "joint_states"
FRANKA_STATES_TOPIC = "franka_state_controller/franka_states"

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
ARM_POSITION_CONTROLLERS = [
    "panda_arm_joint1_position_controller",
    "panda_arm_joint2_position_controller",
    "panda_arm_joint3_position_controller",
    "panda_arm_joint4_position_controller",
    "panda_arm_joint5_position_controller",
    "panda_arm_joint6_position_controller",
    "panda_arm_joint7_position_controller",
]
ARM_EFFORT_CONTROLLERS = [
    "panda_arm_joint1_effort_controller",
    "panda_arm_joint2_effort_controller",
    "panda_arm_joint3_effort_controller",
    "panda_arm_joint4_effort_controller",
    "panda_arm_joint5_effort_controller",
    "panda_arm_joint6_effort_controller",
    "panda_arm_joint7_effort_controller",
]
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
        load_gripper (bool): Whether the gripper was loaded.
        block_gripper (bool): Whether the gripper was blocked.
        robot_control_type (str): The robot control type.
        joint_states (:obj:`sensor_msgs.msg.JointState`): The current joint states.
        franka_states (:obj:`franka_msgs.msg.FrankaState`): The current franka states.
            These give robot specific information about the panda robot.
        joints (dict): The joint that can be controlled.
        gripper_width (float): The gripper width.
        in_collision(bool): Whether the robot is in collision.
        tf_buffer (:obj:`tf2_ros.buffer.Buffer`): Tf buffer object can be used to
            request transforms.
        panda_gazebo (:obj:`ros_gazebo_gym.core.LazyImporter`): Lazy importer for the
            :panda-gazebo:`panda_gazebo <>` package.
        franka_msgs (:obj:`ros_gazebo_gym.core.LazyImporter`): Lazy importer for the
            :franka-gazebo:`franka_msgs <>` package.
    """

    def __init__(  # noqa: C901
        self,
        robot_name_space="",
        robot_EE_link="panda_link8",
        load_gripper=True,
        block_gripper=False,
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
            load_gripper (bool, optional): Whether we want to load the parallel-jaw
                gripper. Defaults to ``True``.
            load_gripper (bool, optional): Whether we want to block the parallel-jaw
                gripper. Defaults to ``False``.
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
        rospy.logdebug("Initialize PandaEnv robot environment...")

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
        self.block_gripper = block_gripper
        self._connection_timeout = CONNECTION_TIMEOUT
        self._joint_traj_action_server_default_step_size = 1
        self._grasping = False if not hasattr(self, "_grasping") else self._grasping
        self._direct_control = (
            False if not hasattr(self, "_direct_control") else self._direct_control
        )
        self._log_step_debug_info = (
            False
            if not hasattr(self, "_log_step_debug_info")
            else self._log_step_debug_info
        )
        self._moveit_set_ee_pose_client_connected = False
        self._moveit_get_ee_pose_joint_config_client_connected = False
        self._arm_joint_traj_control_client_connected = False
        self._set_joint_commands_client_connected = False
        self._set_gripper_width_client_connected = False
        self._fetched_joints = False
        self.__robot_control_type = control_type.lower()
        self.__joints = {}
        self.__in_collision = False

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
                rospy.logerr(
                    "Shutting down '%s' since the Panda Gazebo simulation was not "
                    "started within the set timeout period of %s seconds."
                    % (rospy.get_name(), GAZEBO_SIM_CONNECTION_TIMEOUT)
                )
                sys.exit(0)

        # Validate requested control type.
        if self.robot_control_type not in AVAILABLE_CONTROL_TYPES:
            err_msg = (
                f"Shutting down '{rospy.get_name()}' because control type "
                f"'{control_type}' that was specified is invalid. Please use one of "
                "the following robot control types and try again: "
            )
            for ctrl_type in AVAILABLE_CONTROL_TYPES:
                err_msg += f"\n - {ctrl_type}"
            rospy.logerr(err_msg)
            sys.exit(0)
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
            control_type=control_type_group,
            end_effector=self.robot_EE_link,
            load_gripper=self.load_gripper,
            rviz=show_rviz,
            rviz_file=self._rviz_file if hasattr(self, "_rviz_file") else "",
            disable_franka_gazebo_logs=True,
            log_file=launch_log_file,
            critical=True,
        )

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
            publish_rviz_training_info_overlay=self._load_rviz
            if hasattr(self, "_load_rviz")
            else True,
        )

        ########################################
        # Connect to sensors ###################
        ########################################
        rospy.logdebug("Connecting to sensors.")

        # Create publishers.
        self._in_collision_pub = rospy.Publisher(
            "/ros_gazebo_gym/in_collision", Float32, queue_size=1, latch=True
        )

        # Create joint state and franka state subscriber.
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
        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        ########################################
        # Connect to control services ##########
        ########################################
        rospy.loginfo("Connecting to robot control services.")

        ################################
        # Control switcher #############
        ################################
        # NOTE: Here we use a warpper around the 'control_manager/switch_controller'
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
                rospy.logerr(
                    "Shutting down '%s' since no connection could be established with "
                    "the '%s' service. This service is needed for controlling the "
                    "Panda robot using the '%s' control type."
                    % (
                        rospy.get_name(),
                        moveit_set_ee_pose_srv_topic,
                        self.robot_control_type,
                    )
                )
                sys.exit(0)

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
                rospy.logerr(
                    "Shutting down '%s' since no connection could be established with "
                    "the '%s' service. This service is needed for controlling the "
                    "Panda robot using the '%s' control type."
                    % (
                        rospy.get_name(),
                        set_joint_trajectory_action_srv_topic,
                        self.robot_control_type,
                    )
                )
                sys.exit(0)

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
                    rospy.logerr(
                        "Shutting down '%s' since no connection could be established "
                        "with the '%s' service. This service is needed for controlling "
                        "the arm in 'PROXY' control mode."
                        % (rospy.get_name(), set_joint_commands_srv_topic)
                    )
                    sys.exit(0)
            else:  # Directly publish control commands to controller topics.
                if self.robot_control_type == "position":
                    # Create arm joint position controller publishers.
                    self._arm_joint_position_pub = (
                        self.panda_gazebo.core.GroupPublisher()
                    )
                    for position_controller in ARM_POSITION_CONTROLLERS:
                        self._arm_joint_position_pub.append(
                            rospy.Publisher(
                                "%s/command" % (position_controller),
                                Float64,
                                queue_size=10,
                            )
                        )
                else:
                    # Create arm joint effort publishers.
                    self._arm_joint_effort_pub = self.panda_gazebo.core.GroupPublisher()
                    for effort_controller in ARM_EFFORT_CONTROLLERS:
                        self._arm_joint_effort_pub.append(
                            rospy.Publisher(
                                "%s/command" % (effort_controller),
                                Float64,
                                queue_size=10,
                            )
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
                rospy.logerr(
                    "Shutting down '%s' since no connection could be "
                    "established with the '%s' service. This service is needed "
                    "for controlling the hand in 'PROXY' control mode."
                    % (rospy.get_name(), set_gripper_width_topic)
                )
                sys.exit(0)
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
                rospy.logerr(
                    "Shutting down '%s' since no connection could be "
                    "established with the '%s' action service. This service is "
                    "needed for controlling the hand in 'DIRECT' control mode."
                    % (rospy.get_name(), FRANKA_GRIPPER_COMMAND_TOPIC)
                )
                sys.exit(0)

        # Environment initiation complete message.
        rospy.logdebug("PandaEnv robot environment initialized.")

    ################################################
    # Panda Robot env main methods #################
    ################################################
    def get_ee_pose(self):
        """Returns the end effector EE pose.

        Returns:
            :obj:`geometry_msgs.msg.PoseStamped`: The current end effector pose.

        Raises:
            :obj:`ros_gazebo_gym.errors.EePoseLookupError`: Error thrown when error
                occurred while trying to retrieve the EE pose using the ``get_ee_pose``
                service.
        """
        try:
            # Retrieve EE pose using tf2.
            ee_site_trans = self.tf_buffer.lookup_transform(
                "world", self.robot_EE_link, rospy.Time()
            )

            # Transform trans to pose.
            ee_pose = PoseStamped()
            ee_pose.header = ee_site_trans.header
            ee_pose.pose.orientation = ee_site_trans.transform.rotation
            ee_pose.pose.position = ee_site_trans.transform.translation
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            logwarn_msg = (
                "End effector pose could not be retrieved as "
                + lower_first_char(e.args[0])
            )
            raise EePoseLookupError(
                message="End effector pose could not be retrieved.",
                log_message=logwarn_msg,
            )

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
            # Retrieve EE pose using tf2.
            ee_site_trans = self.tf_buffer.lookup_transform(
                "world", self.robot_EE_link, rospy.Time()
            )

            # Transform trans to pose.
            ee_pose = PoseStamped()
            ee_pose.header = ee_site_trans.header
            ee_pose.pose.orientation = ee_site_trans.transform.rotation
            ee_pose.pose.position = ee_site_trans.transform.translation

            # Convert EE pose to rpy.
            gripper_rpy = get_orientation_euler(ee_pose.pose)  # Yaw, Pitch Roll.
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
        """
        if self._moveit_get_ee_pose_joint_config_client_connected:
            ee_target_pose = Pose()
            ee_target_pose.position.x = ee_pose["x"]
            ee_target_pose.position.y = ee_pose["y"]
            ee_target_pose.position.z = ee_pose["z"]
            ee_target_pose.orientation.x = ee_pose["rx"]
            ee_target_pose.orientation.y = ee_pose["ry"]
            ee_target_pose.orientation.z = ee_pose["rz"]
            ee_target_pose.orientation.w = ee_pose["rw"]
            # Make sure the orientation is normalized.
            ee_target_pose.orientation = normalize_quaternion(
                ee_target_pose.orientation
            )

            # Request and return pose.
            req = self.panda_gazebo.srv.GetEePoseJointConfigRequest()
            req.pose = ee_target_pose
            req.attempts = (
                self._pose_sampling_attempts
                if hasattr(self, "_pose_sampling_attemps")
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
            else:
                return joint_configuration_dict
        else:
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
        if self._moveit_set_ee_pose_client_connected:
            arm_action_space_joints = [
                item
                for item in self._action_space_joints
                if item not in ["gripper_width", "gripper_max_effort"]
            ]
            # Convert float and array joint_commands to dictionary.
            if isinstance(ee_pose, (list, np.ndarray, tuple)) or np.isscalar(ee_pose):
                if np.isscalar(ee_pose):
                    ee_pose = [ee_pose]

                # Create joint_commands dictionary.
                if len(ee_pose) > len(arm_action_space_joints):
                    rospy.logwarn_once(
                        "End effector pose setpoint contains %s values while it "
                        "can only contain %s. As a result only the first %s "
                        "values are used in the arm and gripper control command."
                        % (
                            len(ee_pose),
                            len(arm_action_space_joints),
                            len(ee_pose),
                        )
                    )
                ee_pose = dict(zip(arm_action_space_joints, ee_pose))

            # Create set EE pose request message.
            if isinstance(ee_pose, dict):
                # Fill missing EE pose attributes with curren pose values.
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

                ee_target_pose = Pose()
                ee_target_pose.position.x = ee_pose["x"]
                ee_target_pose.position.y = ee_pose["y"]
                ee_target_pose.position.z = ee_pose["z"]
                ee_target_pose.orientation.x = ee_pose["rx"]
                ee_target_pose.orientation.y = ee_pose["ry"]
                ee_target_pose.orientation.z = ee_pose["rz"]
                ee_target_pose.orientation.w = ee_pose["rw"]
            elif isinstance(ee_pose, PoseStamped):
                ee_target_pose = Pose()
                ee_target_pose.position = ee_pose.pose.position
                ee_target_pose.orientation = ee_pose.pose.orientation
            elif isinstance(ee_pose, Pose):
                ee_target_pose = ee_pose
            elif isinstance(ee_pose, self.panda_gazebo.srv.SetEePoseRequest):
                ee_target = ee_pose
            else:  # If the ee_pose format is not valid.
                rospy.logwarn(
                    "Setting end effector pose failed since the ee_pose you specified "
                    "was given as a %s while the 'set_ee_pose' function only accepts "
                    "a list, Pose and a PoseStamped." % type(ee_pose)
                )
                return False
            ee_target_pose.orientation = normalize_quaternion(
                ee_target_pose.orientation
            )
            if isinstance(ee_target_pose, Pose):
                ee_target = self.panda_gazebo.srv.SetEePoseRequest()
                ee_target.pose = ee_target_pose

            ########################################
            # Set EE pose ##########################
            ########################################
            self._step_debug_logger(
                "Setting end effector pose using the "
                f"'{self._moveit_set_ee_pose_client.resolved_name}' service."
            )
            retval = self._moveit_set_ee_pose_client.call(ee_target)
            if not retval.success:
                logdebug_msg = "End effector pose not set as " + lower_first_char(
                    retval.message
                )
                rospy.logwarn(logdebug_msg)
                return False
            else:
                return True
        else:
            rospy.logwarn(
                "Setting end effector pose failed since the required service '%s' was "
                "not available." % MOVEIT_SET_EE_POSE_TOPIC
            )
            return False

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
                    "Joint positions setpoint contains %s values while it "
                    "can only contain %s. As a result only the first %s "
                    "values are used in the arm and gripper control command."
                    % (
                        len(joint_commands),
                        len(self._action_space_joints),
                        len(joint_commands),
                    )
                )
            joint_commands = dict(zip(self._action_space_joints, joint_commands))

        # Create SetJointCommandsRequest message if PROXY mode.
        if not direct_control:
            if isinstance(joint_commands, dict):
                req = self.panda_gazebo.srv.SetJointCommandsRequest()
                if self._grasping:
                    req.grasping = self._grasping
                req.joint_names = list(joint_commands.keys())
                req.joint_commands = list(joint_commands.values())
                req.control_type = "position"
            elif isinstance(
                joint_commands, self.panda_gazebo.srv.SetJointPositionsRequest
            ):
                req = joint_commands
            else:
                rospy.logwarn(
                    "Setting joint positions failed since the 'joint_commands' "
                    "argument is of the '%s' type while the 'set_joint_positions' "
                    "function only accepts a dictionary, list or a "
                    "'SetJointCommandsRequest' message." % type(joint_commands)
                )
                return False

            # Add wait variables to SetJointCommandsRequest message.
            req.arm_wait = arm_wait
            req.hand_wait = hand_wait
        else:
            if not isinstance(
                joint_commands, (dict, list, np.ndarray, tuple)
            ) or np.isscalar(joint_commands):
                rospy.logwarn(
                    "Setting joint positions failed since the 'joint_commands' "
                    "argument is of the '%s' type while the 'set_joint_positions' "
                    "function only  accepts a dictionary, list or int when in DIRECT "
                    "control mode." % type(joint_commands)
                )
                return False

        ########################################
        # Set joint positions ##################
        ########################################
        self._step_debug_logger(
            "Setting joint positions using {} control mode.".format(
                "DIRECT" if direct_control else "PROXY"
            )
        )
        if not direct_control:  # Use proxy service.
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
        # Fill missing states if joint_commands dictionary is incomplete.
        if list(joint_commands.keys()) != (
            [
                item
                for item in self._action_space_joints
                if item not in ["gripper_width", "gripper_max_effort"]
            ]
            if self.load_gripper and self.block_gripper
            else self._action_space_joints
        ):
            cur_joint_commands = {
                key: val
                for key, val in zip(self.joint_states.name, self.joint_states.position)
                if key in self.joints["arm"]
            }
            if self.load_gripper:
                cur_joint_commands["gripper_width"] = self.gripper_width
                cur_joint_commands["gripper_max_effort"] = (
                    GRASP_FORCE if self._grasping else 0.0
                )

            # Add joint commands.
            cur_joint_commands.update(joint_commands)
            joint_commands = cur_joint_commands

        # Send arm and hand control commands.
        if self.load_gripper and not self.block_gripper:
            gripper_width = joint_commands.pop("gripper_width", None)
            gripper_max_effort = joint_commands.pop("gripper_max_effort", None)
        self._arm_joint_position_pub.publish(
            [Float64(val) for val in joint_commands.values()]
        )
        if arm_wait:
            self._wait_till_arm_control_done(
                control_type="position",
                joint_setpoint=list(joint_commands.values()),
            )
        if self.load_gripper and not self.block_gripper:
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
                    "Joint efforts setpoint contains %s values while it "
                    "can only contain %s. As a result only the first %s "
                    "values are used in the arm and gripper control command."
                    % (
                        len(joint_commands),
                        len(self._action_space_joints),
                        len(joint_commands),
                    )
                )
            joint_commands = dict(zip(self._action_space_joints, joint_commands))

        # Create SetJointCommandsRequest message if PROXY mode.
        if not direct_control:
            if isinstance(joint_commands, dict):
                req = self.panda_gazebo.srv.SetJointCommandsRequest()
                if self._grasping:
                    req.grasping = self._grasping
                req.joint_names = list(joint_commands.keys())
                req.joint_commands = list(joint_commands.values())
                req.control_type = "effort"
            elif isinstance(
                joint_commands, self.panda_gazebo.srv.SetJointEffortsRequest
            ):
                req = joint_commands
            else:
                rospy.logwarn(
                    "Setting joint efforts failed since the 'joint_commands' argument "
                    "is of the '%s' type while the 'set_joint_efforts' function only "
                    "accepts a dictionary, list or a 'SetJointEfforts' message."
                    % type(joint_commands)
                )
                return False

            # Add wait variables to SetJointCommandsRequest message.
            req.arm_wait = arm_wait
            req.hand_wait = hand_wait
        else:
            if not isinstance(
                joint_commands, (dict, list, np.ndarray, tuple)
            ) or np.isscalar(joint_commands):
                rospy.logwarn(
                    "Setting joint efforts failed since the 'joint_commands' argument "
                    "is of the '%s' type while the 'set_joint_efforts' function only "
                    "accepts a dictionary, list or int when in DIRECT control mode."
                    % type(joint_commands)
                )
                return False

        ########################################
        # Set joint efforts ####################
        ########################################
        self._step_debug_logger(
            "Setting joint efforts using {} control mode.".format(
                "DIRECT" if direct_control else "PROXY"
            )
        )
        if not direct_control:  # Use proxy service.
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
        # Fill missing states if joint_commands dictionary is incomplete.
        if list(joint_commands.keys()) != (
            [
                item
                for item in self._action_space_joints
                if item not in ["gripper_width", "gripper_max_effort"]
            ]
            if self.load_gripper and self.block_gripper
            else self._action_space_joints
        ):
            cur_joint_commands = {
                key: val
                for key, val in zip(self.joint_states.name, self.joint_states.effort)
                if key in self.joints["arm"]
            }
            if self.load_gripper:
                cur_joint_commands["gripper_width"] = self.gripper_width
                cur_joint_commands["gripper_max_effort"] = (
                    GRASP_FORCE if self._grasping else 0.0
                )

            # Add joint commands.
            cur_joint_commands.update(joint_commands)
            joint_commands = cur_joint_commands

        # Send arm and hand control commands.
        if self.load_gripper and not self.block_gripper:
            gripper_width = joint_commands.pop("gripper_width", None)
            gripper_max_effort = joint_commands.pop("gripper_max_effort", None)
        self._arm_joint_effort_pub.publish(
            [Float64(val) for val in joint_commands.values()]
        )
        # NOTE: We currently do not have to wait for control efforts to be applied
        # since the 'FrankaHWSim' does not yet implement control latency. Torques
        # are therefore applied instantly.
        # if arm_wait:
        #     self._wait_till_arm_control_done(
        #         control_type="effort",
        #         joint_setpoint=list(joint_commands.values())
        #     )
        if self.load_gripper and not self.block_gripper:
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
                        "Joint trajectory dict contains %s values while it "
                        "can only contain %s. As a result only the first %s "
                        "values are used in the arm and gripper control command."
                        % (
                            len(joint_trajectory),
                            len(arm_action_space_joints),
                            len(joint_trajectory),
                        )
                    )
                joint_trajectory = dict(zip(arm_action_space_joints, joint_trajectory))
            elif (
                isinstance(joint_trajectory, (np.ndarray))
                and joint_trajectory.ndim == 2
            ):
                if joint_trajectory.shape[1] > len(arm_action_space_joints):
                    rospy.logwarn_once(
                        "Joint trajectory dict contains %s values while it "
                        "can only contain %s. As a result only the first %s "
                        "values are used in the arm and gripper control command."
                        % (
                            len(joint_trajectory),
                            len(arm_action_space_joints),
                            len(joint_trajectory),
                        )
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
                    "input argument is of the '%s' type while the "
                    "'set_arm_joint_trajectory' function only accepts a dictionary, "
                    "list or a FollowJointTrajectoryActionGoal message."
                    % type(joint_trajectory)
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
            else:
                return True
        else:
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
        else:
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
                "Please specify a valid control type. Valid values are %s."
                % ("['position', 'effort']")
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
                "control. Please make sure the '%s' controllers that are needed "
                "for '%s' control are initialized."
                % (
                    control_type,
                    ARM_POSITION_CONTROLLERS
                    if control_type == "position"
                    else ARM_EFFORT_CONTROLLERS,
                    control_type,
                )
            )
            return False
        if not any(arm_states_mask):
            rospy.logwarn(
                "Not waiting for control to be completed as no joints appear to be "
                "controlled when using '%s' control. Please make sure the '%s' "
                "controllers that are needed for '%s' control are initialized."
                % (
                    control_type,
                    ARM_POSITION_CONTROLLERS
                    if control_type == "position"
                    else ARM_EFFORT_CONTROLLERS,
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
            rospy.logerr(
                "Shutting down '%s' since the controllers required for '%s' control "
                "were not loaded successfully." % (rospy.get_name(), control_type)
            )
            sys.exit(0)

    @property
    def in_collision(self):
        """Whether the robot is in collision."""
        return self.__in_collision

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
