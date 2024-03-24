"""Contains a small python utility class that makes it easier to interact with the
Gazebo simulator.
"""

import time

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import (
    GetLinkState,
    GetModelState,
    GetPhysicsProperties,
    GetPhysicsPropertiesRequest,
    SetModelConfiguration,
    SetModelConfigurationRequest,
    SetModelState,
    SetModelStateRequest,
    SetPhysicsProperties,
    SetPhysicsPropertiesRequest,
    SpawnModel,
    SpawnModelRequest,
)
from geometry_msgs.msg import Pose, Vector3
from ros_gazebo_gym.common.helpers import (
    deep_update,
    find_gazebo_model_path,
    lower_first_char,
    model_state_msg_2_link_state_dict,
    normalize_quaternion,
)
from ros_gazebo_gym.core.helpers import ros_exit_gracefully
from ros_gazebo_gym.exceptions import (
    GetLinkStateError,
    GetModelStateError,
    GetPhysicsPropertiesError,
    SetModelConfigurationError,
    SetModelStateError,
    SetPhysicsPropertiesError,
    SpawnModelError,
)
from rosgraph_msgs.msg import Clock
from rospy import ServiceException
from rospy.exceptions import ROSException, ROSInterruptException
from rospy_message_converter import message_converter
from std_msgs.msg import Float64
from std_srvs.srv import Empty

# Specify gazebo service topics.
GAZEBO_PAUSE_PHYSICS_TOPIC = "/gazebo/pause_physics"
GAZEBO_UNPAUSE_PHYSICS_TOPIC = "/gazebo/unpause_physics"
GAZEBO_RESET_SIM_TOPIC = "/gazebo/reset_simulation"
GAZEBO_RESET_WORLD_TOPIC = "/gazebo/reset_world"
GAZEBO_CLOCK_TOPIC = "/clock"
GAZEBO_SPAWN_SDF_MODEL_TOPIC = "/gazebo/spawn_sdf_model"
GAZEBO_SPAWN_URDF_MODEL_TOPIC = "/gazebo/spawn_urdf_model"
GAZEBO_LINK_STATES_TOPIC = "/gazebo/link_states"
GAZEBO_MODEL_STATES_TOPIC = "/gazebo/model_states"
GAZEBO_GET_MODEL_STATE_TOPIC = "/gazebo/get_model_state"
GAZEBO_SET_MODEL_STATE_TOPIC = "/gazebo/set_model_state"
GAZEBO_GET_LINK_STATE_TOPIC = "/gazebo/get_link_state"
GAZEBO_SET_MODEL_CONFIGURATION_TOPIC = "/gazebo/set_model_configuration"
GAZEBO_GET_PHYSICS_PROPERTIES_TOPIC = "/gazebo/get_physics_properties"
GAZEBO_SET_PHYSICS_PROPERTIES_TOPIC = "/gazebo/set_physics_properties"

# Script variables.
PHYSICS_UPDATE_RATE = 1000
SERVICES_CONNECTION_TIMEOUTS = 5


class GazeboConnection:
    """Class that contains several methods that can be used to interact with the Gazebo
    simulation.

    Attributes:
        pause_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS service that
            pauses the gazebo simulator.
        unpause_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS service that
            un-pauses the gazebo simulator.
        reset_simulation_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS
            service that resets the gazebo simulator.
        reset_world_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS service
            that resets the gazebo world.
        spawn_sdf_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS service
            that spawns a sdf model.
        spawn_urdf_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS service
            that spawns a urdf model.
        get_model_state_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS
            service used to set get model states.
        set_model_state_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS
            service used to set the model state of a object.
        set_link_state_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS
            service used to set the link states.
        get_link_state_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS
            service used to get the link states.
        set_model_configuration_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`):
            ROS service that sets the configuration of a model.
        get_physics_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS
            service used to retrieve the physics properties.
        set_physics_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS
            service used to set the physics properties.
    """

    def __init__(  # noqa: C901
        self, reset_world_or_sim="WORLD", max_retry=20, retry_rate=5, log_reset=True
    ):
        """Initiate the GazeboConnection instance.

        Args:
            reset_world_or_sim (str, optional): Whether you want to reset the whole
                simulation "SIMULATION" at startup or only the world "WORLD" (object
                positions). Defaults to "WORLD".
            max_retry (int, optional): How many times a command to the simulator is
                retried before giving up. Defaults to ``30``.
            retry_rate (int, optional): The rate at which the retry is done. Defaults to
                ``2`` (i.e. 0.5 seconds).
            log_reset (bool, optional): Whether we want to print a log statement when
                the world/simulation is reset. Defaults to ``True``.
        """
        rospy.logwarn("Initialize GazeboConnection utility class...")
        self._reset_world_or_sim = reset_world_or_sim
        self._log_reset = log_reset
        self._max_retry = max_retry
        self._retry_rate = retry_rate
        self._physics_update_rate = Float64(PHYSICS_UPDATE_RATE)
        self.__time = 0.0

        # Connect to link_state and model_state topics.
        rospy.Subscriber(
            GAZEBO_LINK_STATES_TOPIC, ModelStates, self._link_states_cb, queue_size=1
        )
        rospy.Subscriber(
            GAZEBO_MODEL_STATES_TOPIC, ModelStates, self._model_states_cb, queue_size=1
        )

        # Connect to gazebo services.
        try:
            rospy.logdebug("Connecting to '%s' service." % GAZEBO_PAUSE_PHYSICS_TOPIC)
            rospy.wait_for_service(
                GAZEBO_PAUSE_PHYSICS_TOPIC,
                timeout=SERVICES_CONNECTION_TIMEOUTS,
            )
            self.pause_proxy = rospy.ServiceProxy(GAZEBO_PAUSE_PHYSICS_TOPIC, Empty)
            rospy.logdebug("Connected to '%s' service!" % GAZEBO_PAUSE_PHYSICS_TOPIC)
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GAZEBO_PAUSE_PHYSICS_TOPIC
            )
        try:
            rospy.logdebug("Connecting to '%s' service." % GAZEBO_UNPAUSE_PHYSICS_TOPIC)
            rospy.wait_for_service(
                GAZEBO_UNPAUSE_PHYSICS_TOPIC,
                timeout=SERVICES_CONNECTION_TIMEOUTS,
            )
            self.unpause_proxy = rospy.ServiceProxy(GAZEBO_UNPAUSE_PHYSICS_TOPIC, Empty)
            rospy.logdebug("Connected to '%s' service!" % GAZEBO_UNPAUSE_PHYSICS_TOPIC)
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GAZEBO_UNPAUSE_PHYSICS_TOPIC
            )
        try:
            rospy.logdebug("Connecting to '%s' service." % GAZEBO_RESET_SIM_TOPIC)
            rospy.wait_for_service(
                GAZEBO_RESET_SIM_TOPIC,
                timeout=SERVICES_CONNECTION_TIMEOUTS,
            )
            self.reset_simulation_proxy = rospy.ServiceProxy(
                GAZEBO_RESET_SIM_TOPIC, Empty
            )
            rospy.logdebug("Connected to '%s' service!" % GAZEBO_RESET_SIM_TOPIC)
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn("Failed to connect to '%s' service!" % GAZEBO_RESET_SIM_TOPIC)
        try:
            rospy.logdebug("Connecting to '%s' service." % GAZEBO_RESET_WORLD_TOPIC)
            rospy.wait_for_service(
                GAZEBO_RESET_WORLD_TOPIC,
                timeout=SERVICES_CONNECTION_TIMEOUTS,
            )
            self.reset_world_proxy = rospy.ServiceProxy(GAZEBO_RESET_WORLD_TOPIC, Empty)
            rospy.logdebug("Connected to '%s' service!" % GAZEBO_RESET_WORLD_TOPIC)
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GAZEBO_RESET_WORLD_TOPIC
            )
        try:
            rospy.logdebug("Connecting to '%s' service." % GAZEBO_SPAWN_SDF_MODEL_TOPIC)
            rospy.wait_for_service(
                GAZEBO_SPAWN_SDF_MODEL_TOPIC,
                timeout=SERVICES_CONNECTION_TIMEOUTS,
            )
            self.spawn_sdf_proxy = rospy.ServiceProxy(
                GAZEBO_SPAWN_SDF_MODEL_TOPIC, SpawnModel
            )
            rospy.logdebug("Connected to '%s' service!" % GAZEBO_SPAWN_SDF_MODEL_TOPIC)
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GAZEBO_SPAWN_SDF_MODEL_TOPIC
            )
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % GAZEBO_SPAWN_URDF_MODEL_TOPIC
            )
            rospy.wait_for_service(
                GAZEBO_SPAWN_URDF_MODEL_TOPIC,
                timeout=SERVICES_CONNECTION_TIMEOUTS,
            )
            self.spawn_urdf_proxy = rospy.ServiceProxy(
                GAZEBO_SPAWN_URDF_MODEL_TOPIC, SpawnModel
            )
            rospy.logdebug("Connected to '%s' service!" % GAZEBO_SPAWN_URDF_MODEL_TOPIC)
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GAZEBO_SPAWN_URDF_MODEL_TOPIC
            )
        try:
            rospy.logdebug("Connecting to '%s' service." % GAZEBO_GET_MODEL_STATE_TOPIC)
            rospy.wait_for_service(
                GAZEBO_GET_MODEL_STATE_TOPIC,
                timeout=SERVICES_CONNECTION_TIMEOUTS,
            )
            self.get_model_state_proxy = rospy.ServiceProxy(
                GAZEBO_GET_MODEL_STATE_TOPIC, GetModelState
            )
            rospy.logdebug("Connected to '%s' service!" % GAZEBO_GET_MODEL_STATE_TOPIC)
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GAZEBO_GET_MODEL_STATE_TOPIC
            )
        try:
            rospy.logdebug("Connecting to '%s' service." % GAZEBO_SET_MODEL_STATE_TOPIC)
            rospy.wait_for_service(
                GAZEBO_SET_MODEL_STATE_TOPIC,
                timeout=SERVICES_CONNECTION_TIMEOUTS,
            )
            self.set_model_state_proxy = rospy.ServiceProxy(
                GAZEBO_SET_MODEL_STATE_TOPIC, SetModelState
            )
            rospy.logdebug("Connected to '%s' service!" % GAZEBO_SET_MODEL_STATE_TOPIC)
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GAZEBO_SET_MODEL_STATE_TOPIC
            )
        try:
            rospy.logdebug("Connecting to '%s' service." % GAZEBO_GET_LINK_STATE_TOPIC)
            rospy.wait_for_service(
                GAZEBO_GET_LINK_STATE_TOPIC,
                timeout=SERVICES_CONNECTION_TIMEOUTS,
            )
            self.get_link_state_proxy = rospy.ServiceProxy(
                GAZEBO_GET_LINK_STATE_TOPIC, GetLinkState
            )
            rospy.logdebug("Connected to '%s' service!" % GAZEBO_GET_LINK_STATE_TOPIC)
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % GAZEBO_GET_LINK_STATE_TOPIC
            )
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % GAZEBO_SET_MODEL_CONFIGURATION_TOPIC
            )
            rospy.wait_for_service(
                GAZEBO_SET_MODEL_CONFIGURATION_TOPIC,
                timeout=SERVICES_CONNECTION_TIMEOUTS,
            )
            self.set_model_configuration_proxy = rospy.ServiceProxy(
                GAZEBO_SET_MODEL_CONFIGURATION_TOPIC, SetModelConfiguration
            )
            rospy.logdebug(
                "Connected to '%s' service!" % GAZEBO_SET_MODEL_CONFIGURATION_TOPIC
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!"
                % GAZEBO_SET_MODEL_CONFIGURATION_TOPIC
            )

        # Setup physics properties control service.
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % GAZEBO_GET_PHYSICS_PROPERTIES_TOPIC
            )
            rospy.wait_for_service(
                GAZEBO_GET_PHYSICS_PROPERTIES_TOPIC,
                timeout=SERVICES_CONNECTION_TIMEOUTS,
            )
            self.get_physics_proxy = rospy.ServiceProxy(
                GAZEBO_GET_PHYSICS_PROPERTIES_TOPIC, GetPhysicsProperties
            )
            rospy.logdebug(
                "Connected to '%s' service!" % GAZEBO_GET_PHYSICS_PROPERTIES_TOPIC
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!"
                % GAZEBO_GET_PHYSICS_PROPERTIES_TOPIC
            )
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % GAZEBO_SET_PHYSICS_PROPERTIES_TOPIC
            )
            rospy.wait_for_service(
                GAZEBO_SET_PHYSICS_PROPERTIES_TOPIC,
                timeout=SERVICES_CONNECTION_TIMEOUTS,
            )
            self.set_physics_proxy = rospy.ServiceProxy(
                GAZEBO_SET_PHYSICS_PROPERTIES_TOPIC, SetPhysicsProperties
            )
            rospy.logdebug(
                "Connected to '%s' service!" % GAZEBO_SET_PHYSICS_PROPERTIES_TOPIC
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!"
                % GAZEBO_SET_PHYSICS_PROPERTIES_TOPIC
            )

        # Connect to clock.
        rospy.Subscriber(GAZEBO_CLOCK_TOPIC, Clock, self._clock_cb, queue_size=1)

        # Reset the simulation.
        self.reset_sim()

        # We always pause the simulation, important for legged robots learning.
        self.pause_sim()

        rospy.logwarn("GazeboConnection utility class initialised.")

    def pause_sim(self):
        """Pause the simulation."""
        rospy.logdebug("PAUSING service found...")
        paused_done, counter, warned = False, 0, False
        while not paused_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    rospy.logdebug("PAUSING service calling...")
                    self.pause_proxy()
                    paused_done = True
                    rospy.logdebug("PAUSING service calling...DONE")
                except rospy.ServiceException:
                    if not warned:
                        rospy.logerr(
                            "/gazebo/pause_physics service call failed retrying "
                            f"{self._max_retry} times."
                        )
                        warned = True
                    counter += 1
                    time.sleep(1 / self._retry_rate)
            else:
                rospy.logerr(
                    f"Maximum retries done ({self._max_retry}), please check Gazebo "
                    "pause service and try again."
                )
                ros_exit_gracefully(
                    shutdown_msg=f"Shutting down {rospy.get_name()}", exit_code=1
                )
        rospy.logdebug("PAUSING finished")

    def unpause_sim(self):
        """Unpauses the simulation."""
        rospy.logdebug("UNPAUSING start")
        unpaused_done, counter, warned = False, 0, False
        while not unpaused_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    rospy.logdebug("UNPAUSING service calling...")
                    self.unpause_proxy()
                    unpaused_done = True
                    rospy.logdebug("UNPAUSING service calling...DONE")
                except rospy.ServiceException:
                    if not warned:
                        rospy.logerr(
                            "/gazebo/unpause_physics service call failed retrying "
                            f"{self._max_retry} times."
                        )
                        warned = True
                    counter += 1
                    time.sleep((1 / self._retry_rate))
            else:
                rospy.logerr(
                    f"Maximum retries done ({self._max_retry}), please check Gazebo "
                    "unpause service and try again."
                )
                ros_exit_gracefully(
                    shutdown_msg=f"Shutting down {rospy.get_name()}", exit_code=1
                )
        rospy.logdebug("UNPAUSING finished")

    def _reset_simulation(self):
        """Calls the ROS reset simulation service."""
        rospy.wait_for_service(GAZEBO_RESET_SIM_TOPIC)
        try:
            self.reset_simulation_proxy()
        except rospy.ServiceException:
            print(f"{self.reset_simulation_proxy.resolved_name} service call failed")

    def _reset_world(self):
        """Resets the world (NOT THE WHOLE SIMULATION)."""
        rospy.wait_for_service(GAZEBO_RESET_WORLD_TOPIC)
        try:
            self.reset_world_proxy()
        except rospy.ServiceException:
            print(f"{self.reset_world_proxy.resolved_name} service call failed")

    def reset_sim(self):
        """Reset the simulation or the world.

        .. note::
            Implemented like this since in some simulations, when reset the simulation
            the systems that work with TF break. In this case we ONLY resets the object
            position, not the entire simulation.
        """
        if self._reset_world_or_sim == "SIMULATION":
            if self._log_reset:
                rospy.logwarn("SIMULATION RESET")
            self._reset_simulation()
        elif self._reset_world_or_sim == "WORLD":
            if self._log_reset:
                rospy.logwarn("WORLD RESET")
            self._reset_world()
        elif self._reset_world_or_sim == "NO_RESET_SIM":
            rospy.logerr("NO RESET SIMULATION SELECTED")
        else:
            rospy.logerr("WRONG Reset Option:" + str(self._reset_world_or_sim))

    #############################################
    # Properties/functions for retrieving/ ######
    # setting gazebo environment properties #####
    #############################################
    def get_model_state(self, model_name):
        """Retrieve the current state of a model.

        Args:
            model_name (str): The name of the model for which you want to retrieve the
                state.

        Returns:
            :obj:`numpy.ndarray`: The pose of the model.

        Raises:
            :obj:`ros_gazebo_gym.exceptions.GetModelStateError`: Thrown when the model
                state retrieval failed.
        """
        model_state = self.get_model_state_proxy(model_name, "world")
        if model_state.success:
            return np.array(
                [
                    model_state.pose.position.x,
                    model_state.pose.position.y,
                    model_state.pose.position.z,
                    model_state.pose.orientation.x,
                    model_state.pose.orientation.y,
                    model_state.pose.orientation.z,
                ]
            )
        else:
            logwarn_msg = f"Model state of '{model_name}' model could not be retrieved."
            rospy.logwarn(logwarn_msg)
            raise GetModelStateError(
                message=logwarn_msg, details=model_state.status_message
            )

    def set_model_state(self, model_state):
        """Sets the state of a model.

        Args:
            model_state (:obj:`gazebo_msgs.msg.SetModelState`): The model_state object.

        Returns:
            bool: Boolean specifying whether the model state was set successfully.

        Raises:
            :obj:`ros_gazebo_gym.exceptions.SetModelStateError`: Thrown when the model
                state could not be set.
        """
        rospy.logdebug("setting '%s' model state." % model_state.model_name)
        retval = self.set_model_state_proxy.call(SetModelStateRequest(model_state))
        if not retval.success:
            logwarn_msg = "Model state model could not be set."
            rospy.logwarn(logwarn_msg)
            raise SetModelStateError(logwarn_msg)
        return retval.success

    def get_link_state(self, link_name):
        """Retrieve the current state of a model.

        Args:
            link_name (str): The name of the robot link.

        Returns:
            :obj:`numpy.ndarray`: The pose of the robot link.

        Raises:
            GetLinkStateError: Thrown when the link state retrieval failed.
        """
        link_state = self.get_link_state_proxy(link_name, "world")
        if link_state.success:
            return np.array(
                [
                    link_state.pose.position.x,
                    link_state.pose.position.y,
                    link_state.pose.position.z,
                    link_state.pose.orientation.x,
                    link_state.pose.orientation.y,
                    link_state.pose.orientation.z,
                ]
            )
        else:
            logwarn_msg = f"Model state of '{link_name}' model could not be retrieved."
            rospy.logwarn(logwarn_msg)
            raise GetLinkStateError(
                message=logwarn_msg, details=link_state.status_message
            )

    def set_model_configuration(
        self, model_name, joint_names, joint_positions, pause=True
    ):
        """Sets the configuration of a model.

        Args:
            model_name (string): Model to set the configuration for.
            joint_names (list): The joint names for which you want to set the
                configuration.
            joint_positions (list): The joint positions you want to set.
            pause (bool, optional): Pause the simulation while setting the model pose.
                Defaults to ``True``.

        Returns:
            bool: Boolean specifying whether the model configuration was set
                successfully.

        Raises:
            :obj:`ros_gazebo_gym.exceptions.SetModelConfigurationError`: Thrown when the
                model configuration could not be set.
        """
        rospy.logdebug("setting '%s' model state." % model_name)
        if len(joint_names) != len(joint_positions):
            logwarn_msg = (
                "Model configuration model could not be set since the 'joint_names' "
                "and 'joint_position' are unequal in length."
            )
            rospy.logwarn(logwarn_msg)
            raise SetModelConfigurationError(logwarn_msg)

        # Set joint position.
        if pause:
            self.pause_sim()
        retval = self.set_model_configuration_proxy.call(
            SetModelConfigurationRequest(
                model_name=model_name,
                joint_names=joint_names,
                joint_positions=joint_positions,
            )
        )
        if pause:
            self.unpause_sim()
        if not retval.success:
            logwarn_msg = "Model configuration model could not be set."
            rospy.logwarn(logwarn_msg)
            raise SetModelConfigurationError(logwarn_msg)
        return retval.success

    def get_physics_properties(self):
        """Retrieve physics properties from gazebo.

        Returns:
            :obj:`gazebo_msgs.srv.GetPhysicsPropertiesResponse`: Physics properties
                message.

        Raises:
            :obj:`ros_gazebo_gym.errors.GetPhysicsPropertiesError`: Thrown when
                something goes wrong while trying to retrieve the physics properties.
        """
        physics_properties_msg = self.physics_properties
        if not physics_properties_msg.success:
            logwarn_msg = "Physics properties could not be retrieved."
            rospy.logwarn(logwarn_msg)
            raise GetPhysicsPropertiesError(
                message=logwarn_msg, details=physics_properties_msg.status_message
            )
        return physics_properties_msg

    def set_physics_properties(self, **kwargs):
        """Change physics properties of the gazebo physics engine. These properties have
        to be supplied as a keyword argument.

        .. tip::
            You can use the :obj:`GazeboConnection.set_physics_proxy` If you want to
            send a :obj:`gazebo_msgs.srv.SetPhysicsProperties` message directly.

        Args:
            **kwargs: Keyword arguments specifying the physics properties you want to
                set.

        Raises:
            SetPhysicsPropertiesError: Thrown when something goes wrong while setting
                the physics properties.
        """
        physics_properties_dict = message_converter.convert_ros_message_to_dictionary(
            self.physics_properties
        )
        del (
            physics_properties_dict["pause"],
            physics_properties_dict["success"],
            physics_properties_dict["status_message"],
        )
        physics_properties_msg = message_converter.convert_dictionary_to_ros_message(
            "gazebo_msgs/SetPhysicsProperties",
            deep_update(physics_properties_dict, fixed=True, **kwargs),
            kind="request",
        )
        retval = self.set_physics_proxy.call(physics_properties_msg)
        if not retval.success:
            logwarn_msg = "Physics engine could not be updated."
            rospy.logwarn(logwarn_msg)
            raise SetPhysicsPropertiesError(
                message=logwarn_msg, details=retval.status_message
            )

    def _update_gravity_call(self, x, y, z):
        """Updates the simulator gravity.

        Args:
            x (float): Gravity vector x coordinate.
            y (float): Gravity vector y coordinate.
            z (float): Gravity vector z coordinate.

        Raises:
            :obj:`ros_gazebo_gym.exceptions.SetPhysicsPropertiesError`: Thrown when the
                physics properties could not be set.
        """
        self.pause_sim()

        # Retrieve current physics properties.
        try:
            get_physics_properties = self.physics_properties
        except ServiceException:
            logwarn_msg = (
                "Failed to retrieve physics properties when trying to update gravity. "
                "As a result, gravity was not updated."
            )
            rospy.logwarn(logwarn_msg)
            return

        # Create set physics properties request message.
        set_physics_request = SetPhysicsPropertiesRequest()
        for key in [
            attr for attr in dir(get_physics_properties) if not attr.startswith("_")
        ]:  # Set all properties to the current physics properties.
            try:
                setattr(set_physics_request, key, getattr(get_physics_properties, key))
            except AttributeError:
                pass
        set_physics_request.gravity = Vector3(x=x, y=y, z=z)
        rospy.logdebug(str(set_physics_request.gravity))

        # Send physic change request.
        result = self.set_physics_proxy(set_physics_request)
        rospy.logdebug(
            "Gravity Update Result=="
            + str(result.success)
            + ",message=="
            + str(result.status_message)
        )
        self.unpause_sim()

    def change_gravity(self, x, y, z):
        """Changes the gravity vector

        Args:
            x (float): Gravity vector x coordinate.
            y (float): Gravity vector y coordinate.
            z (float): Gravity vector z coordinate.
        """
        self._update_gravity_call(x, y, z)

    def spawn_object(  # noqa: C901
        self,
        object_name,
        model_name,
        models_folder_path,
        pose=None,
    ):
        """Spawns a object from the model directory into gazebo.

        Args:
            object_name (str): The name you want the model to have.
            model_name (str): The model type (The name of the xml file you want to use).
            models_folder_path (str): The folder in which you want to search for the
                models.
            pose (:obj:`geometry_msgs.msg.Pose`, optional): The pose of the model, by
                default :py:class:`geometry_msgs.msg.Pose`.

        Returns:
            bool: A boolean specifying whether the model was successfully spawned.

        Raises:
            :obj:`ros_gazebo_gym.exceptions.SpawnModelError`: When model was not spawned
                successfully.
        """
        if not pose:  # Use default pose if none was given.
            pose = Pose()
            pose.orientation = normalize_quaternion(pose.orientation)

        # Check if model is already present.
        if object_name in self.model_states.keys():
            rospy.logwarn(
                f"A model with model name '{model_name}' already exists. Please check "
                "if this is the right model."
            )
            return False

        # Find model xml.
        rospy.logdebug("Looking for '%s' model file." % model_name)
        model_xml, extension = find_gazebo_model_path(model_name, models_folder_path)
        if not model_xml:  # If model file was not found.
            logwarn_msg = (
                f"Spawning model '{model_name}' as '{object_name}' failed since the "
                "sdf/urd model file was not found. Please make sure you added the "
                f"model sdf/urdf file to the '{models_folder_path}' folder."
            )
            rospy.logwarn(logwarn_msg)
            raise SpawnModelError(message=logwarn_msg)

        # Load content from the model xml file.
        xml_file = open(model_xml, "r")
        model_xml_content = xml_file.read()

        # Create spawn model request message.
        rospy.logdebug("Spawning '%s' model as '%s'." % (model_name, object_name))
        spawn_model_req = SpawnModelRequest(
            model_name=object_name,
            model_xml=model_xml_content,
            initial_pose=pose,
            reference_frame="world",
        )

        # Request model spawn from sdf or urdf spawn service.
        rospy.logdebug("Spawning model '%s' as '%s'." % (model_name, object_name))
        spawn_done = False
        counter = 0
        if extension == "sdf":  # Use sdf service.
            while not spawn_done and not rospy.is_shutdown():
                if counter < self._max_retry:
                    try:
                        rospy.logdebug("SPAWNING service calling...")
                        retval = self.spawn_sdf_proxy.call(spawn_model_req)
                        spawn_done = True
                        rospy.logdebug("SPAWNING service calling...DONE")
                        return retval
                    except rospy.ServiceException as e:
                        logwarn_msg = (
                            f"Spawning model '{model_name}' as '{object_name}' failed "
                            f"since {lower_first_char(e.args[0])}."
                        )
                        rospy.logwarn(logwarn_msg)
                        raise SpawnModelError(
                            message=logwarn_msg, details={"exception": e}
                        )
        else:
            while not spawn_done and not rospy.is_shutdown():
                if counter < self._max_retry:
                    try:
                        rospy.logdebug("SPAWNING service calling...")
                        retval = self.spawn_urdf_proxy.call(spawn_model_req)
                        spawn_done = True
                        rospy.logdebug("SPAWNING service calling...DONE")
                        return retval
                    except rospy.ServiceException as e:
                        logwarn_msg = (
                            f"Spawning model '{model_name}' as '{object_name}' failed "
                            f"since {lower_first_char(e.args[0])}."
                        )
                        rospy.logwarn(logwarn_msg)
                        raise SpawnModelError(
                            message=logwarn_msg, details={"exception": e}
                        )

    def _link_states_cb(self, data):
        """Link states subscriber callback function.

        Args:
            data (:obj:`gazebo_msgs.msg.ModelStates`): The data that is
                returned by the subscriber.
        """
        self.link_states = model_state_msg_2_link_state_dict(data)

    def _model_states_cb(self, data):
        """Model states subscriber callback function.

        Args:
            data (:obj:`gazebo_msgs.msg.ModelStates`): The data that is
                returned by the subscriber.
        """
        self.model_states = model_state_msg_2_link_state_dict(data)

    def _clock_cb(self, data):
        """Gazebo clock subscriber callback function.

        Args:
            data (:obj:`gazebo_msgs.msg.ModelStates`): The data that is
                returned by the subscriber.
        """
        self.__time = data.clock.to_time()

    @property
    def physics_properties(self):
        """Retrieves the physics properties from gazebo."""
        return self.get_physics_proxy(GetPhysicsPropertiesRequest())

    @property
    def time(self):
        """Retrieves the Gazebo time."""
        return self.__time
