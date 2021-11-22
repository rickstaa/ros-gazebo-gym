#!/usr/bin/env python3
"""Contains a small python API class that makes it easier to interact with the Gazebo
simulator.
"""

import rospy
from gazebo_msgs.srv import (
    GetPhysicsProperties,
    GetPhysicsPropertiesRequest,
    SetPhysicsProperties,
    SetPhysicsPropertiesRequest,
)
from openai_ros.common.functions import (
    deep_update,
)
from openai_ros.exceptions import (
    GetPhysicsPropertiesError,
    SetPhysicsPropertiesError,
)
from rospy.exceptions import ROSException, ROSInterruptException
from rospy_message_converter import message_converter
from std_msgs.msg import Float64
from std_srvs.srv import Empty

# Specify gazebo service topics
GAZEBO_PAUSE_PHYSICS_TOPIC = "/gazebo/pause_physics"
GAZEBO_UNPAUSE_PHYSICS_TOPIC = "/gazebo/unpause_physics"
GAZEBO_RESET_SIM_TOPIC = "/gazebo/reset_simulation"
GAZEBO_RESET_WORLD_TOPIC = "/gazebo/reset_world"
GAZEBO_GET_PHYSICS_PROPERTIES_TOPIC = "/gazebo/get_physics_properties"
GAZEBO_SET_PHYSICS_PROPERTIES_TOPIC = "/gazebo/set_physics_properties"

# Script variables
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
        get_physics_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): Ros
            service used to retrieve the physics properties.
        set_physics_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): Ros
            service used to set the physics properties.
    """

    def __init__(  # noqa: C901
        self,
        reset_world_or_sim="WORLD",
        max_retry=20
    ):
        """Initiate the GazeboConnection instance.

        Args:
            reset_world_or_sim (str, optional): Wether you want to reset the whole
                simulation "SIMULATION" at startup or only the world "WORLD" (object
                positions). Defaults to "WORLD".
            max_retry (int, optional): How many times a command to the simulator is
                retried before giving up. Defaults to ``20``.
        """
        self._reset_world_or_sim = reset_world_or_sim
        self._max_retry = max_retry
        self._physics_update_rate = Float64(PHYSICS_UPDATE_RATE)

        # Connect to gazebo services
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

        # Setup physics properties control service
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

        # Reset the simulation
        self.reset_sim()

        # We always pause the simulation, important for legged robots learning
        self.pause_sim()

    def pause_sim(self):
        """Pause the simulation."""
        rospy.logdebug("PAUSING service found...")
        paused_done = False
        counter = 0
        while not paused_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    rospy.logdebug("PAUSING service calling...")
                    self.pause_proxy()
                    paused_done = True
                    rospy.logdebug("PAUSING service calling...DONE")
                except rospy.ServiceException:
                    counter += 1
                    rospy.logerr("/gazebo/pause_physics service call failed")
            else:
                error_message = (
                    "Maximum retries done"
                    + str(self._max_retry)
                    + ", please check Gazebo pause service"
                )
                rospy.logerr(error_message)
                assert False, error_message

        rospy.logdebug("PAUSING finished")

    def unpause_sim(self):
        """Unpauses the simulation."""
        rospy.logdebug("UNPAUSING start")
        unpaused_done = False
        counter = 0
        while not unpaused_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    rospy.logdebug("UNPAUSING service calling...")
                    self.unpause_proxy()
                    unpaused_done = True
                    rospy.logdebug("UNPAUSING service calling...DONE")
                except rospy.ServiceException:
                    counter += 1
                    rospy.logerr(
                        "/gazebo/unpause_physics service call failed...Retrying "
                        + str(counter)
                    )
            else:
                error_message = (
                    "Maximum retries done"
                    + str(self._max_retry)
                    + ", please check Gazebo unpause service"
                )
                rospy.logerr(error_message)
                assert False, error_message
        rospy.logdebug("UNPAUSING finished")

    def _reset_simulation(self):
        """Calls the ROS reset simulation service."""
        rospy.wait_for_service("/gazebo/reset_simulation")
        try:
            self.reset_simulation_proxy()
        except rospy.ServiceException:
            print("/gazebo/reset_simulation service call failed")

    def _reset_world(self):
        """Resets the world (NOT THE WHOLE SIMULATION)."""
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_world_proxy()
        except rospy.ServiceException:
            print("/gazebo/reset_world service call failed")

    def reset_sim(self):
        """Reset the simulation or the world.

        .. note::
            Implemented like this since in some simulations, when reset the simulation
            the systems that work with TF break. In this case we ONLY resets the object
            position, not the entire simulation.
        """
        if self._reset_world_or_sim == "SIMULATION":
            rospy.logerr("SIMULATION RESET")
            self._reset_simulation()
        elif self._reset_world_or_sim == "WORLD":
            rospy.logerr("WORLD RESET")
            self._reset_world()
        elif self._reset_world_or_sim == "NO_RESET_SIM":
            rospy.logerr("NO RESET SIMULATION SELECTED")
        else:
            rospy.logerr("WRONG Reset Option:" + str(self._reset_world_or_sim))

    def _update_gravity_call(self):
        """Updates the simulator gravity property."""
        self.pause_sim()

        # Create physic change message
        set_physics_request = SetPhysicsPropertiesRequest()
        set_physics_request.time_step = 1 / self._physics_update_rate.data
        set_physics_request.max_update_rate = self._physics_update_rate.data
        set_physics_request.gravity = self._gravity
        set_physics_request.ode_config = self._ode_config
        rospy.logdebug(str(set_physics_request.gravity))

        # Send physic change request
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
        self._gravity.x = x
        self._gravity.y = y
        self._gravity.z = z
        self._update_gravity_call()

    #############################################
    # Properties/functions for retrieving #######
    # gazebo env information ####################
    #############################################
    def get_physics_properties(self):
        """Retrieve physics properties from gazebo.

        Returns:
            :obj:`gazebo_msgs.srv.GetPhysicsPropertiesResponse`: Physics properties
                message.

        Raises:
            :obj:`openai_ros.errors.GetPhysicsPropertiesError`: Thrown when something
                goes wrong while trying to retrieve the physics properties.
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
            You can use the `:obj:`GazeboConnection.` If you want to send a
            :obj:`gazebo_msgs.srv.SetPhysicsProperties` message directly.

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

    @property
    def physics_properties(self):
        """Retrieves the physics properties from gazebo."""
        return self.get_physics_proxy(GetPhysicsPropertiesRequest())
