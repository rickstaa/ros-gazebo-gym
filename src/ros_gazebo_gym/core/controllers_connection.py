"""Contains a python utility class that makes it easier to interact with
`ros_control <https://wiki.ros.org/ros_control>`_ controllers.
"""

import rospy
from controller_manager_msgs.srv import (
    ListControllers,
    ListControllersRequest,
    SwitchController,
    SwitchControllerRequest,
)
from ros_gazebo_gym.common.helpers import flatten_list
from ros_gazebo_gym.core.helpers import ros_exit_gracefully
from rosgraph_msgs.msg import Clock
from rospy.exceptions import ROSException, ROSInterruptException
from std_srvs.srv import Empty

# Script settings.
CONNECTION_TIMEOUT = 10


class ControllersConnection:
    """Class that contains several methods that can be used to interact with the
    `ros_control <https://wiki.ros.org/ros_control>`_ controllers.

    Attributes:
        list_service_name (str): The name of the controller list service.
        list_service (:obj:`rospy.impl.tcpros_service.ServiceProxy`): The controller
            list service.
        switch_service_name (str): The name of the controller switch service.
        switch_service (:obj:`rospy.impl.tcpros_service.ServiceProxy`): The controller
            switch service.
    """

    def __init__(self, namespace="", controllers_list=None):  # noqa: C901
        """Initialize the ControllersConnection instance.

        Args:
            namespace (str, optional): The namespace on which the robot controllers can
                be found. Defaults to ``""``.
            controllers_list (list, optional): A list with currently available
                controllers to look for. Defaults to ``None``, which means that the
                class will try to retrieve all the running controllers.
        """
        rospy.logwarn("Initialize ControllersConnection utility class...")
        self._controller_list = controllers_list
        self._paused_controllers = []
        self._gazebo_paused_check_timeout = 0.2
        self._list_controllers_service_name = (
            f"{namespace}/controller_manager/list_controllers"
        )
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % self._list_controllers_service_name
            )
            rospy.wait_for_service(
                self._list_controllers_service_name,
                timeout=CONNECTION_TIMEOUT,
            )
            self._list_controllers_proxy = rospy.ServiceProxy(
                self._list_controllers_service_name, ListControllers
            )
            rospy.logdebug(
                "Connected to '%s' service!" % self._list_controllers_service_name
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            error_msg = (
                f"Shutting down '{rospy.get_name()}' since no connection could be "
                f"established with the {self._list_controllers_service_name} service!"
            )
            ros_exit_gracefully(shutdown_msg=error_msg, exit_code=1)
        self._switch_controller_service_name = (
            f"{namespace}/controller_manager/switch_controller"
        )
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % self._switch_controller_service_name
            )
            rospy.wait_for_service(
                self._switch_controller_service_name,
                timeout=CONNECTION_TIMEOUT,
            )
            self._switch_controller_proxy = rospy.ServiceProxy(
                self._switch_controller_service_name, SwitchController
            )
            rospy.logdebug(
                "Connected to '%s' service!" % self._switch_controller_service_name
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            error_msg = (
                f"Shutting down '{rospy.get_name()}' since no connection could be "
                f"established with the {self._list_controllers_service_name} service!"
            )
            ros_exit_gracefully(shutdown_msg=error_msg, exit_code=1)
        self._gazebo_pause_service_name = "/gazebo/pause_physics"
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % self._gazebo_pause_service_name
            )
            rospy.wait_for_service(
                self._gazebo_pause_service_name,
                timeout=CONNECTION_TIMEOUT,
            )
            self._pause_proxy = rospy.ServiceProxy(
                self._gazebo_pause_service_name, Empty
            )
            rospy.logdebug(
                "Connected to '%s' service!" % self._gazebo_pause_service_name
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % self._gazebo_pause_service_name
            )
        self._gazebo_unpause_service_name = "/gazebo/unpause_physics"
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % self._gazebo_unpause_service_name
            )
            rospy.wait_for_service(
                self._gazebo_unpause_service_name,
                timeout=CONNECTION_TIMEOUT,
            )
            self._unpause_proxy = rospy.ServiceProxy(
                self._gazebo_unpause_service_name, Empty
            )
            rospy.logdebug(
                "Connected to '%s' service!" % self._gazebo_unpause_service_name
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logwarn(
                "Failed to connect to '%s' service!" % self._gazebo_unpause_service_name
            )

        rospy.logwarn("ControllersConnection utility class initialised.")

    def switch_controllers(
        self, controllers_on, controllers_off, strictness=1, timeout=0.0
    ):
        """Function used to switch controllers on and off.

        Args:
            controllers_on (list): The controllers you want to turn on.
            controllers_off (list): The controllers you want to turn off.
            strictness (int, optional): Whether the switching will fail if anything goes
                wrong. Defaults to ``1``.
            timeout (float): The timeout before the request is cancelled. Defaults to
                ``0.0`` meaning no timeout.

        Returns:
            bool: Boolean specifying whether the switch was successful.
        """
        rospy.wait_for_service(self._switch_controller_service_name)
        try:
            switch_request_object = SwitchControllerRequest()
            switch_request_object.start_controllers = controllers_on
            switch_request_object.stop_controllers = controllers_off
            switch_request_object.strictness = strictness
            switch_request_object.timeout = timeout
            switch_result = self._switch_controller_proxy(switch_request_object)
            # NOTE: When a ROS Gazebo simulation is present and it is paused we briefly
            # unpause it. This is needed for the `controller_manager/switch_controller`
            # service to work.
            if not switch_result.ok and self.gazebo and self.gazebo_paused:
                self._unpause_proxy()
                switch_result = self._switch_controller_proxy(switch_request_object)
                self._pause_proxy()
            rospy.logdebug("Switch Result==>" + str(switch_result.ok))
            return switch_result.ok
        except rospy.ServiceException:
            print(self._switch_controller_service_name + " service call failed")
            return None

    def reset_controllers(self, timeout=1.0):
        """Resets the currently running controllers by turning them off and on.

        Args:
            timeout (float): The timeout before the request is cancelled. Defaults to
                ``0.0`` meaning no timeout.
        """
        # Try to find all running controllers controller_list was not supplied.
        if self._controller_list is None:
            list_controllers_msg = self._list_controllers_proxy.call(
                ListControllersRequest()
            )
            controllers_list = [
                controller.name
                for controller in list_controllers_msg.controller
                if controller.state == "running"
            ]
        else:
            controllers_list = self._controller_list

        # Reset the running controllers.
        reset_result = False
        rospy.logdebug("Deactivating controllers")
        result_off_ok = self.switch_controllers(
            controllers_on=[], controllers_off=controllers_list, timeout=timeout
        )
        if result_off_ok:
            rospy.logdebug("Re-activating controllers")
            result_on_ok = self.switch_controllers(
                controllers_on=controllers_list, controllers_off=[]
            )
            if result_on_ok:
                rospy.logdebug("Controllers reset==>" + str(controllers_list))
                reset_result = True
            else:
                rospy.logdebug("result_on_ok==>" + str(result_on_ok))
        else:
            rospy.logdebug("result_off_ok==>" + str(result_off_ok))

        return reset_result

    def pause_controllers(self, controller_list=None, filter_list=[]):
        """Pauses controllers.

        Args:
            controller_list (list, optional): The controllers you want to pause.
                Defaults to ``None``, which means that the class will pause all the
                running controllers.
            filter_list (list, optional): The controllers you want to ignore when
                pausing. Defaults to ``[]``.
        """
        # Retrieve all running controllers if controller_list was not supplied.
        if controller_list is None:
            if self._controller_list is None:
                list_controllers_msg = self._list_controllers_proxy.call(
                    ListControllersRequest()
                )
                controllers_list = [
                    controller.name
                    for controller in list_controllers_msg.controller
                    if controller.state == "running"
                ]
            else:
                controllers_list = self._controller_list

        # Filter out the controllers that should be ignored.
        controllers_list = [
            controller
            for controller in controllers_list
            if controller not in filter_list
        ]

        # Pause the running controllers.
        pause_result = False
        rospy.logdebug("Pausing controllers")
        self._paused_controllers = controllers_list
        result_off_ok = self.switch_controllers(
            controllers_on=[], controllers_off=controllers_list
        )
        if result_off_ok:
            rospy.logdebug("Controllers paused==>" + str(controllers_list))
            pause_result = True
        else:
            rospy.logdebug("result_off_ok==>" + str(result_off_ok))

        return pause_result

    def unpause_controllers(self):
        """Unpauses all the paused controllers.

        Returns:
            bool: Boolean specifying whether the unpause was successful.
        """
        if self._paused_controllers:
            unpause_result = False
            rospy.logdebug("Unpausing controllers")
            result_on_ok = self.switch_controllers(
                controllers_on=self._paused_controllers, controllers_off=[]
            )
            if result_on_ok:
                rospy.logdebug(
                    "Controllers unpaused==>" + str(self._paused_controllers)
                )
                unpause_result = True
                self._paused_controllers = []
            else:
                rospy.logdebug("result_on_ok==>" + str(result_on_ok))

            return unpause_result
        else:
            rospy.logwarn("No controllers to unpause!")
            return False

    @property
    def controllers_list(self):
        """Returns the list of available controllers."""
        return self._controllers_list

    @controllers_list.setter
    def controllers_list(self, new_controllers_list):
        """Updates the list of available controllers."""
        self._controllers_list = new_controllers_list

    @property
    def gazebo(self):
        """Returns whether a ROS Gazebo simulation is running."""
        return any(
            ["/gazebo" in topic for topic in flatten_list(rospy.get_published_topics())]
        )

    @property
    def gazebo_paused(self):
        """Returns whether the Gazebo simulation is paused."""
        if self.gazebo and any(
            ["/clock" in topic for topic in flatten_list(rospy.get_published_topics())]
        ):
            try:
                rospy.wait_for_message(
                    "/clock", Clock, timeout=self._gazebo_paused_check_timeout
                )
                return False
            except ROSException:
                return True

        return False
