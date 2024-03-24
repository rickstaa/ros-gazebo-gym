"""Template file used that can be used to create a new Robot environment. It contains a
Python class that specifies the robot to use on the task. It provides the complete
integration between the Gazebo simulation of the robot and the gymnasium library, so
obtaining **SENSOR** information from the robot or sending **ACTIONS** to it are
transparent to the gymnasium library and you, the developer. For more information, see
the `ros_gazebo_gym <https://rickstaa.dev/ros-gazebo-gym>`_ documentation.

Source code
-----------

.. literalinclude:: ../../../../../templates/template_my_robot_env.py
   :language: python
   :linenos:
   :lines: 16-
"""

from ros_gazebo_gym.robot_gazebo_env import RobotGazeboEnv


class MyRobotEnv(RobotGazeboEnv):
    """Superclass for all Robot environments."""

    def __init__(self):
        """Initializes a new Robot environment."""
        # Setup internal robot environment variables (controllers, namespace etc.).
        # NOTE: If the controllers_list is not set all the currently running controllers
        # will be reset by the ControllersList class.
        # TODO: Change the controller list and robot namespace.
        self.controllers_list = [
            "my_robot_controller1",
            "my_robot_controller2",
            # ...,
            "my_robot_controllerX",
        ]
        self.robot_name_space = "my_robot_namespace"
        reset_controls_bool = True or False

        # Initialize the gazebo environment.
        super(MyRobotEnv, self).__init__(
            controllers_list=self.controllers_list,
            robot_name_space=self.robot_name_space,
            reset_controls=reset_controls_bool,
        )

    ################################################
    # Overload Gazebo env virtual methods ##########
    ################################################
    # NOTE: Methods that need to be implemented as they are called by the robot and
    # Gazebo environments.
    def _check_all_systems_ready(self):
        """Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        # TODO: Implement the logic that checks if all sensors and actuators are ready.
        return True

    ################################################
    # Robot environment internal methods ###########
    ################################################
    # NOTE: Here you can add additional helper methods that are used in the robot env.

    ################################################
    # Robot env main methods #######################
    ################################################
    # NOTE: Contains methods that the task environment will need.
