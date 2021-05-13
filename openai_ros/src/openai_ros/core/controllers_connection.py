#!/usr/bin/env python3
"""Contains a small python API class that makes it easier to interact with the
`ros_control <https://wiki.ros.org/ros_control>` controllers.
"""

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest


class ControllersConnection:
    """Class that contains several methods that can be used to interact with the
    ros_control controllers.

    Attributes:
        controllers_list (list): List with currently available controllers.
        switch_service_name (str): The name of the controller switch service.
        switch_service (): The controller switch service.
    """  # TODO: Set right type

    def __init__(self, namespace, controllers_list):
        """Initialize the ControllersConnection instance.

        Args:
            namespace (str): The namespace on which the robot controllers can be found.
            controllers_list (list): A list with currently available controllers to
                look for.
        """
        rospy.logwarn("Start Init ControllersConnection")
        self.controllers_list = controllers_list
        self.switch_service_name = (
            "/" + namespace + "/controller_manager/switch_controller"
        )
        self.switch_service = rospy.ServiceProxy(
            self.switch_service_name, SwitchController
        )
        rospy.logwarn("END Init ControllersConnection")

    def switch_controllers(self, controllers_on, controllers_off, strictness=1):
        """Function used to switch controllers on and off.

        Args:
            controllers_on ([type]): [description]
            controllers_off ([type]): [description]
            strictness (int, optional): [description]. Defaults to 1.

        Returns:
            [type]: [description]
        """  # TODO: Docstring
        rospy.wait_for_service(self.switch_service_name)

        try:
            switch_request_object = SwitchControllerRequest()
            switch_request_object.start_controllers = controllers_on
            switch_request_object.start_controllers = controllers_off
            switch_request_object.strictness = strictness

            switch_result = self.switch_service(switch_request_object)
            """
            [controller_manager_msgs/SwitchController]
            int32 BEST_EFFORT=1
            int32 STRICT=2
            string[] start_controllers
            string[] stop_controllers
            int32 strictness
            ---
            bool ok
            """
            rospy.logdebug("Switch Result==>" + str(switch_result.ok))

            return switch_result.ok

        except rospy.ServiceException:
            print(self.switch_service_name + " service call failed")

            return None

    def reset_controllers(self):
        """Resets the currently used controllers by turning them off and on.
        """
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
        """Update the available controllers list.
        """
        self.controllers_list = new_controllers_list
