#!/usr/bin/env python3
"""Contains a small python API class that makes it easier to interact with the
`ros_control <https://wiki.ros.org/ros_control>` controllers.
"""

import sys
import rospy
from controller_manager_msgs.srv import (
    SwitchController,
    SwitchControllerRequest,
    ListControllers,
    ListControllersRequest,
)
from rospy.exceptions import ROSException, ROSInterruptException

# Script settings
CONNECTION_TIMEOUT = 10


class ControllersConnection:
    """Class that contains several methods that can be used to interact with the
    ros_control controllers.

    Attributes:
        controllers_list (list): List with currently available controllers.
        list_service_name (str): The name of the controller list service.
        list_service (:obj:`rospy.impl.tcpros_service.ServiceProxy`): The controller
            list service.
        switch_service_name (str): The name of the controller switch service.
        switch_service (:obj:`rospy.impl.tcpros_service.ServiceProxy`): The controller
            switch service.
    """

    def __init__(self, namespace, controllers_list=None):
        """Initialize the ControllersConnection instance.

        Args:
            namespace (str): The namespace on which the robot controllers can be found.
            controllers_list (list, optional): A list with currently available
                controllers to look for. Defaults to ``None``, which means that the class
                will try to retrieve all the running controllers.
        """
        rospy.logwarn("Start Init ControllersConnection")
        self.controllers_list = controllers_list
        self.list_controllers_service_name = (
            "/" + namespace + "/controller_manager/list_controllers"
        )
        try:
            rospy.logdebug(
                "Connecting to '%s' service." % self.list_controllers_service_name
            )
            rospy.wait_for_service(
                self.list_controllers_service_name,
                timeout=CONNECTION_TIMEOUT,
            )
            self.list_service = rospy.ServiceProxy(
                self.list_controllers_service_name, ListControllers
            )
            rospy.logdebug(
                "Connected to '%s' service!" % self.list_controllers_service_name
            )
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logerr(
                f"Shutting down '{rospy.get_name()}' since no connection could be "
                f"established with the {self.list_controllers_service_name} service!"
            )
            sys.exit(0)
        self.switch_service_name = (
            "/" + namespace + "/controller_manager/switch_controller"
        )
        try:
            rospy.logdebug("Connecting to '%s' service." % self.switch_service_name)
            rospy.wait_for_service(
                self.switch_service_name,
                timeout=CONNECTION_TIMEOUT,
            )
            self.switch_service = rospy.ServiceProxy(
                self.switch_service_name, SwitchController
            )
            rospy.logdebug("Connected to '%s' service!" % self.switch_service_name)
        except (rospy.ServiceException, ROSException, ROSInterruptException):
            rospy.logerr(
                f"Shutting down '{rospy.get_name()}' since no connection could be "
                f"established with the {self.list_controllers_service_name} service!"
            )
            sys.exit(0)

        rospy.logwarn("END Init ControllersConnection")

    def switch_controllers(self, controllers_on, controllers_off, strictness=1):
        """Function used to switch controllers on and off.

        Args:
            controllers_on (list): The controllers you want to turn on.
            controllers_off (list): The controllers you want to turn off.
            strictness (int, optional): Wether the switching will fail if anything goes
                wrong. Defaults to 1.

        Returns:
            bool: Boolean specifying whether the switch was successfull.
        """
        rospy.wait_for_service(self.switch_service_name)
        try:
            switch_request_object = SwitchControllerRequest()
            switch_request_object.start_controllers = controllers_on
            switch_request_object.start_controllers = controllers_off
            switch_request_object.strictness = strictness
            switch_result = self.switch_service(switch_request_object)
            rospy.logdebug("Switch Result==>" + str(switch_result.ok))
            return switch_result.ok
        except rospy.ServiceException:
            print(self.switch_service_name + " service call failed")
            return None

    def reset_controllers(self):
        """Resets the currently running controllers by turning them off and on."""
        # Try to find all running controllers controller_list was not supplied
        if self.controllers_list is None:
            list_controllers_msg = self.list_service.call(ListControllersRequest())
            self.controllers_list = [
                controller.name
                for controller in list_controllers_msg.controller
                if controller.state == "running"
            ]

        # Reset the running controllers
        reset_result = False
        result_off_ok = self.switch_controllers(
            controllers_on=[], controllers_off=self.controllers_list
        )
        rospy.logdebug("Deactivated controllers")
        if result_off_ok:
            rospy.logdebug("Activating controllers")
            result_on_ok = self.switch_controllers(
                controllers_on=self.controllers_list, controllers_off=[]
            )
            if result_on_ok:
                rospy.logdebug("Controllers reset==>" + str(self.controllers_list))
                reset_result = True
            else:
                rospy.logdebug("result_on_ok==>" + str(result_on_ok))
        else:
            rospy.logdebug("result_off_ok==>" + str(result_off_ok))
        return reset_result

    def update_controllers_list(self, new_controllers_list):
        """Update the available controllers list."""
        self.controllers_list = new_controllers_list
