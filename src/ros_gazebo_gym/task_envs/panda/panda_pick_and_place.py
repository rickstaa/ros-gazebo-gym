"""An ROS Panda pick and place gymnasium environment.

.. image:: /images/panda/panda_pick_and_place_env.png
   :alt: Panda pick and place environment

This environment is an extension of the :class:`~ros_gazebo_gym.task_envs.panda.panda_reach.PandaReachEnv` task environment,
sharing most features such as the action spaces. Notable distinctions are detailed below.

Observation space:
    The observation space was extended with the 11 observations:

    - Object x position.
    - Object y position.
    - Object z position.
    - x distance between object and end-effector.
    - y distance between object and end-effector.
    - z distance between object and end-effector.
    - Object yaw rotation.
    - Object pitch rotation.
    - Object roll rotation.
    - Object translational velocity.
    - Object rotational velocity.

Goal:
    In this environment the agent has to learn to lift a block up to reach the desired
    goal position. It was based on the :gymnasium-robotics:`FetchPickAndPlace-v2 <envs/fetch/pick_and_place/>`
    gymnasium environment.

.. admonition:: Configuration
    :class: important

    The configuration files for this environment are found in the
    :ros-gazebo-gym:`panda task environment config folder <blob/noetic/src/ros_gazebo_gym/task_envs/panda/config/panda_pick_and_place.yaml>`.
"""  # noqa: E501

import time
from pathlib import Path

import numpy as np
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, Vector3
from gymnasium import utils
from ros_gazebo_gym.common.helpers import get_orientation_euler, normalize_quaternion
from ros_gazebo_gym.core.helpers import ros_exit_gracefully
from ros_gazebo_gym.core.ros_launcher import ROSLauncher
from ros_gazebo_gym.exceptions import SetModelStateError, SpawnModelError
from ros_gazebo_gym.task_envs.panda.markers.cube_marker import CubeMarker
from ros_gazebo_gym.task_envs.panda.markers.frame_origin_marker import FrameOriginMarker
from ros_gazebo_gym.task_envs.panda.panda_reach import PandaReachEnv
from rospy import ROSException, ROSInterruptException

try:
    from panda_gazebo.srv import AddBox, AddBoxRequest
except ImportError:
    pass

# Specify topics and other script variables.
CONNECTION_TIMEOUT = 5
TARGET_OBJECT_SPAWN_TIMEOUT = 60
TARGET_OBJECT_POSE_PUBLISH_RATE = 60
MOVEIT_ADD_BOX_TOPIC = "panda_moveit_planner_server/planning_scene/add_box"
CONFIG_FILE_PATH = "config/panda_pick_and_place.yaml"


#################################################
# Panda pick and place environment Class ########
#################################################
class PandaPickAndPlaceEnv(PandaReachEnv, utils.EzPickle):
    """Classed used to create a Panda pick and place environment.

    Attributes:
        object_marker_class (:obj:`visualization_msgs.msg.Marker`): The RViz marker
            class used for displaying the object. Can be overwritten by child
            environments to change the visualization of the object.
        object_frame_name (str): The name used for the object tf frame.
    """

    def __init__(
        self,
        config_path=CONFIG_FILE_PATH,
        gazebo_world_launch_file="start_pick_and_place_world.launch",
        *args,
        **kwargs,
    ):
        """Initializes a Panda pick and place task environment.

        Args:
            config_path (str, optional): Path where the environment configuration
                value are found. The path is resolved relative to the
                :class:`~ros_gazebo_gym.task_envs.panda.panda_reach` class file.
            gazebo_world_launch_file (str, optional): Name of the launch file that loads
                the gazebo world. Currently only the launch files inside the
                `panda_gazebo <https://github.com/rickstaa/panda-gazebo>`_ package are
                supported. Defaults to ``start_pick_and_place_world.launch``.
            *args: Arguments passed to the
                :class:`~ros_gazebo_gym.task_envs.panda.PandaReachEnv` super class.
            **kwargs: Keyword arguments that are passed to the
                :class:`~ros_gazebo_gym.task_envs.panda.PandaReachEnv` super class.
        """
        rospy.logdebug("Initialize Panda pick and place environment.")
        utils.EzPickle.__init__(
            **locals()
        )  # Makes sure the env is pickable when it wraps C++ code.
        ROSLauncher.initialize()  # Makes sure roscore is running and ROS is initialized.

        # Create storage variables.
        self._prev_time = rospy.get_time()
        self._prev_grip_position = np.zeros(3)
        self._prev_object_position = np.zeros(3)
        self._prev_object_rot = np.zeros(3)

        super().__init__(
            config_path=config_path,
            gazebo_world_launch_file=gazebo_world_launch_file,
            *args,
            **kwargs,
        )

        # Setup MoveIt platform add service.
        moveit_add_box_srv_topic = f"{self.robot_name_space}/{MOVEIT_ADD_BOX_TOPIC}"
        try:
            rospy.logdebug("Connecting to '%s' service." % moveit_add_box_srv_topic)
            rospy.wait_for_service(
                moveit_add_box_srv_topic,
                timeout=CONNECTION_TIMEOUT,
            )
            self._moveit_add_box_client = rospy.ServiceProxy(
                moveit_add_box_srv_topic, AddBox
            )
            rospy.logdebug("Connected to '%s' service!" % moveit_add_box_srv_topic)
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % moveit_add_box_srv_topic
            )

        self._setup_object()
        # IMPROVE: Currently the platform properties are hardcoded these can be made
        # dynamic by requesting them via a custom made Gazebo plugin.
        self._add_platform_to_moveit_scene()

        # Create object publishers.
        self.object_marker_class = CubeMarker
        self.object_frame_name = "cube"
        rospy.logdebug("Creating RViz object marker publisher.")
        self._object_marker_pub = rospy.Publisher(
            "/ros_gazebo_gym/object", CubeMarker, queue_size=1, latch=True
        )
        rospy.logdebug("RViz object marker publisher created.")
        rospy.logdebug("Creating RViz object frame marker publisher.")
        self._object_frame_marker_pub = rospy.Publisher(
            "/ros_gazebo_gym/object_frame", FrameOriginMarker, queue_size=1, latch=True
        )
        rospy.logdebug("RViz object frame marker publisher created.")
        rospy.Timer(
            rospy.Duration(1.0 / TARGET_OBJECT_POSE_PUBLISH_RATE),
            self._object_marker_pub_cb,
        )

        rospy.logdebug("Panda pick and place environment initialized.")

    ################################################
    # Task environment internal methods ############
    ################################################
    # NOTE: Here you can add additional helper methods that are used in the task env.

    def _object_marker_pub_cb(self, event=None):
        """Callback function that publishes a RViz marker representing the current
        object position.

        Args:
            event (:obj:`rospy.TimerEvent`): The timer event object.

        .. important::
            The object size is currently hardcoded. Please change it when you change
            the object in the :obj:`panda_gazebo` package.
        """
        # IMPROVE: Currently the object properties are hardcoded these can be made
        # dynamic by requesting them via a custom made Gazebo plugin.

        # Publish object frame tf.
        object_pose = self.object_pose
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = self.object_frame_name
        tf_msg.transform.translation.x = object_pose.position.x
        tf_msg.transform.translation.y = object_pose.position.y
        tf_msg.transform.translation.z = object_pose.position.z
        tf_msg.transform.rotation.x = object_pose.orientation.x
        tf_msg.transform.rotation.y = object_pose.orientation.y
        tf_msg.transform.rotation.z = object_pose.orientation.z
        tf_msg.transform.rotation.w = object_pose.orientation.w
        tf2_ros.TransformBroadcaster().sendTransform(tf_msg)

        # Publish object RViz marker.
        self._object_marker_msg = self.object_marker_class(
            frame_id=self.object_frame_name
        )
        self._object_marker_pub.publish(self._object_marker_msg)

        # Also display marker for object frame position.
        object_frame_origin_marker_msg = FrameOriginMarker(
            frame_id=self.object_frame_name
        )
        self._object_frame_marker_pub.publish(object_frame_origin_marker_msg)

    def _add_platform_to_moveit_scene(self):
        """Add the platform to the MoveIt planning scene."""
        # IMPROVE: Currently the platform properties are hardcoded these can be made
        # dynamic by requesting them via a custom made Gazebo plugin.
        add_platform_req = AddBoxRequest(
            name=self._platform_name, pose=self._platform_pose, size=self._platform_size
        )
        resp = self._moveit_add_box_client(add_platform_req)
        if not resp.success:
            rospy.logerr(
                "The platform was not added successfully to the MoveIt planning scene. "
                " As a result, MoveIt is unaware of the platform's existence and may "
                "plan states that are in collision."
            )

    def _setup_object(self):  # noqa: C901
        """Makes sure the object is present in the gazebo simulation."""
        rospy.logdebug("Setting initial object pose.")

        # Try to spawn object if not yet present.
        if not self.contains_object:
            init_obj_pose = Pose(
                position=Vector3(
                    x=self._fixed_object_pose["x"],
                    y=self._fixed_object_pose["y"],
                    z=self._fixed_object_pose["z"],
                ),
                orientation=normalize_quaternion(
                    Quaternion(
                        x=self._fixed_object_pose["rx"],
                        y=self._fixed_object_pose["ry"],
                        z=self._fixed_object_pose["rz"],
                        w=self._fixed_object_pose["rw"],
                    )
                ),
            )
            try:
                self.gazebo.spawn_object(
                    object_name=self._object_name,
                    model_name=self._object_name,
                    models_folder_path=Path(__file__)
                    .joinpath(
                        "../../../../../../"
                        "rosdeps/panda_gazebo/panda_gazebo/resources/models"
                    )
                    .resolve(),
                    pose=init_obj_pose,
                )

                # Wait till the object is found in the model state.
                start_time = time.time()
                while time.time() - start_time < TARGET_OBJECT_SPAWN_TIMEOUT:
                    rospy.logwarn_once(
                        f"Waiting for '{self._object_name}' to be spawned."
                    )
                    if self.contains_object:
                        break
                if not self.contains_object:
                    raise SpawnModelError(
                        f"Object '{self._object_name}' was not spawned within the "
                        f"'{TARGET_OBJECT_SPAWN_TIMEOUT}' second time limit."
                    )
            except SpawnModelError:
                err_msg = (
                    f"Shutting down {rospy.get_name()} since the task environment "
                    f"target object `{self._object_name}` was not spawned successfully."
                )
                ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)

    def _set_init_obj_pose(self):
        """Sets the object to its (random) initial pose.

        Returns:
            bool: Success boolean.
        """
        # Set a random object pose.
        if self._object_sampling_strategy.lower() == "global":
            # Retrieve x,y positions of the current and initial object pose.
            obj_pose = self.gazebo.model_states[self._object_name]["pose"]
            obj_xy_positions = np.array(
                [
                    self.gazebo.model_states[self._object_name]["pose"].position.x,
                    self.gazebo.model_states[self._object_name]["pose"].position.y,
                ]
            )

            # Sample an object initial object (x, y) position.
            while (
                np.linalg.norm(
                    np.array([obj_pose.position.x, obj_pose.position.y])
                    - obj_xy_positions
                )
                < self._object_sampling_distance_threshold
            ):  # Sample till is different enough from the current object pose.
                obj_xy_positions = self.np_random.uniform(
                    [
                        self._object_sampling_bounds["x_min"],
                        self._object_sampling_bounds["y_min"],
                    ],
                    [
                        self._object_sampling_bounds["x_max"],
                        self._object_sampling_bounds["y_max"],
                    ],
                    size=2,
                )

            # Set init object pose.
            obj_model_state = ModelState()
            obj_model_state.model_name = self._object_name
            obj_model_state.pose.position.x = obj_xy_positions[0]
            obj_model_state.pose.position.y = obj_xy_positions[1]
            obj_model_state.pose.position.z = obj_pose.position.z
            rospy.logdebug("Init object pose:")
            rospy.logdebug(obj_pose)
            try:
                retval = self.gazebo.set_model_state(obj_model_state)
            except SetModelStateError:
                err_msg = (
                    f"Shutting down '{rospy.get_name()}' since the state of the grasp "
                    "object could not be set."
                )
                ros_exit_gracefully(shutdown_msg=err_msg, exit_code=1)

            # Return result.
            if not retval:
                rospy.logwarn("setting initial object position failed.")
            return retval

        return True

    def _get_elapsed_time(self):
        """Returns the elapsed time since the last time this function was called."""
        current_time = rospy.get_time()
        dt = current_time - self._prev_time
        self._prev_time = current_time
        return dt

    @property
    def contains_object(self):
        """Checks whether the simulation contains the object.

        Returns:
            bool: Whether the object was found in the simulation.
        """
        return self._object_name in self.gazebo.model_states

    @property
    def object_position(self):
        """Retrieves the current object position."""
        object_pose = self.object_pose
        return np.array(
            [
                object_pose.position.x,
                object_pose.position.y,
                object_pose.position.z,
            ]
        )

    @property
    def object_rot(self):
        """Retrieves the current object rotation."""
        object_rot_resp = get_orientation_euler(self.object_pose)
        return np.array([object_rot_resp.y, object_rot_resp.p, object_rot_resp.r])

    @property
    def object_pose(self):
        """Retrieves the current object pose."""
        return self.gazebo.model_states[self._object_name]["pose"]

    @property
    def _platform_pose(self):
        """Retrieves the current platform pose.

        .. warning::
            Please be aware that the name of the platform has been hardcoded. As a
            result this function breaks when the platform is renamed in the gazebo
            world.
        """
        return self.gazebo.model_states[self._platform_name]["pose"]

    ################################################
    # Overload Reach environment methods ###########
    ################################################
    def _get_params(self, ns="panda_pick_and_place"):  # noqa: C901
        """Retrieve task environment configuration parameters from parameter server.

        Args:
            ns (str, optional): The namespace on which the parameters are found.
                Defaults to "panda_pick_and_place".
        """
        super()._get_params(ns=ns)
        try:
            # Retrieve control variables.
            self._platform_name = rospy.get_param(f"/{ns}/training/platform_name")
            self._platform_size = rospy.get_param(f"/{ns}/training/platform_size")
            self._object_name = rospy.get_param(f"/{ns}/training/object_name")
            self._object_sampling_strategy = rospy.get_param(
                f"/{ns}/object_sampling/strategy"
            )
            self._object_sampling_distance_threshold = rospy.get_param(
                f"/{ns}/object_sampling/distance_threshold"
            )
            self._visualize_object_bounds = rospy.get_param(
                f"/{ns}/object_sampling/visualize_object_bounds"
            )
            self._fixed_object_pose = rospy.get_param(
                f"/{ns}/object_sampling/fixed_pose"
            )
            self._object_sampling_bounds = rospy.get_param(
                f"/{ns}/object_sampling/bounds"
            )
        except KeyError as e:
            rospy.logerr(
                f"Parameter '{e.args[0]}' could not be retrieved from the parameter "
                "server. Please make sure this parameter is present in the Panda task "
                f"environment configuration file '{self._config_file_path}' and try "
                "again."
            )
            ros_exit_gracefully(
                shutdown_msg=f"Shutting down {rospy.get_name()}.", exit_code=1
            )

    def _get_obs(self):
        """Get robot state observation.

        Returns:
            tuple containing:

                - observation array (:obj:`list`):
                    List of observations:
                        - End effector x position
                        - End effector y position
                        - End effector z position
                        - Object x position.
                        - Object y position.
                        - Object z position.
                        - x distance between object and ee.
                        - y distance between object and ee.
                        - z distance between object and ee.
                        - End effector joints positions
                        - End effector joints velocities
                        - Object yaw rotation.
                        - Object pitch rotation.
                        - Object roll rotation.
                        - Object translational velocity.
                        - Object rotational velocity.
                - achieved_goal (:obj:`object`): The goal that was achieved during
                    execution.
                - desired_goal (:obj:`object`): The desired goal that we asked the agent
                    to attempt to achieve.
        """
        obs_dict = super()._get_obs()
        obs = obs_dict["observation"]
        desired_goal = obs_dict["desired_goal"]

        # Unpack required observations.
        ee_position = obs[:3]
        ee_state = obs[3:5]
        ee_vel = obs[5:]

        # Retrieve information about the object.
        if self.contains_object:
            # Retrieve gripper velocity.
            dt = self._get_elapsed_time()
            grip_velp = (
                ee_position - self._prev_grip_position
            ) / dt  # Velocity(position) = Distance/Time.

            # Get object position.
            object_position = self.object_position

            # Get object orientation.
            object_rot = self.object_rot

            # Get object velocity.
            object_velp = (
                object_position - self._prev_object_position
            ) / dt  # Velocity(position) = Distance/Time.
            object_velr = (
                object_rot - self._prev_object_rot
            ) / dt  # Velocity(rotation) = Rotation/Time.

            # Get relative position and velocity.
            object_rel_position = object_position - ee_position
            object_velp -= grip_velp
        else:
            object_position = object_rot = object_velp = object_velr = (
                object_rel_position
            ) = np.zeros(3)

        # Save current gripper and object positions.
        self._prev_grip_position = ee_position
        self._prev_object_position = object_position
        self._prev_object_rot = object_rot

        # Get achieved goal.
        achieved_goal = object_position

        # Concatenate observations.
        obs = np.concatenate(
            [
                ee_position,
                object_position.ravel(),
                object_rel_position.ravel(),
                ee_state,
                object_rot.ravel(),
                object_velp.ravel(),
                object_velr.ravel(),
                ee_vel,
            ],
            dtype=self._observation_space_dtype,
        )
        achieved_goal = np.array(ee_position, dtype=self._observation_space_dtype)
        desired_goal = self.goal.astype(self._observation_space_dtype)

        return {
            "observation": obs,
            "achieved_goal": achieved_goal,
            "desired_goal": desired_goal,
        }

    def _init_env_variables(self):
        """Inits variables needed to be initialized each time we reset at the start
        of an episode.
        """
        self._set_init_obj_pose()

        # Reset storage variables.
        self._prev_time = rospy.get_time()
        self._prev_grip_position = super()._get_obs()["observation"][:3]
        self._prev_object_position = self.object_position
        self._prev_object_rot = self.object_rot

        # Sample and visualize goal.
        self.goal = self._sample_goal()
        if self._visualize_target:
            self._visualize_goal()
