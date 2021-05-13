"""Template file used that can be used to create a new Robot environment.

This Robot environment contains a python class that specifies the robot to use on the
task. It provides the complete integration between the Gazebo simulation of the robot
and the OpenAI algorithm environments, so obtaining *SENSOR* information from the robot
or sending *ACTIONS* to it are ROS transparent to the OpenAI algorithms and to you, the
developer. For more information see the `openai_ros <https://wiki.ros.org/openai_ros>`_
documentation.
"""
from openai_ros import robot_gazebo_env


class MyRobotEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all Robot environments."""

    def __init__(self):
        """Initializes a new Robot environment."""

        # Initialize the Gazebo parent environment
        # TODO: Change the controller list and robot namespace
        self.controllers_list = [
            "my_robot_controller1",
            "my_robot_controller2",
            # ...,
            "my_robot_controllerX",
        ]
        self.robot_name_space = "my_robot_namespace"
        reset_controls_bool = True or False
        super(MyRobotEnv, self).__init__(
            controllers_list=self.controllers_list,
            robot_name_space=self.robot_name_space,
            reset_controls=reset_controls_bool,
        )

    #############################################
    # Overload Gazebo env virtual methods #######
    #############################################
    # NOTE: Methods that need to be implemented as they are called by the Gazebo
    # parent environment.
    def _check_all_systems_ready(self):
        """Checks that all the sensors, publishers and other simulation systems are
        operational.

        Raises:
            NotImplementedError: Thrown when not overloaded by the robot environment.
        """
        # TODO
        return True

    def _env_setup(self, initial_qpos):
        """Initial configuration of the environment. Can be used to configure initial state
        and extract information from the simulation.

        Args:
            initial_qpos (np.ndarray): The initial agent pose (generalized coordinates).

        Raises:
            NotImplementedError: Thrown when not overloaded by the robot environment.
        """
        # TODO: OPTIONAL
        return True

    #############################################
    # Robot environment internal methods ########
    #############################################
    # NOTE: Here you can add additional helper methods that are used in the robot env

    #############################################
    # Robot env virtual methods #################
    #############################################
    # NOTE: Below are methods that the TrainingEnvironment will need to define here as
    # virtual because they will be used in RobotGazeboEnv. These methods need to be
    # overloaded in the TrainingEnvironment.
    def _set_init_pose(self):
        """Sets the Robot in its init pose.

        Raises:
            NotImplementedError: Thrown when not overloaded by the task environment.
        """
        raise NotImplementedError()

    def _get_obs(self):
        """Returns the observation.

        Raises:
            NotImplementedError: Thrown when not overloaded by the task environment.
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.

        Raises:
            NotImplementedError: Thrown when not overloaded by the task environment.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.

        Args:
            action (np.ndarray): The action you want to set.

        Raises:
            NotImplementedError: Thrown When the method was not overloaded by the task
                environment.
        """
        raise NotImplementedError()

    def _is_done(self, observations):
        """Indicates whether or not the episode is done (the robot has fallen for
        example).

        Args:
            observations (np.ndarray): The observations.

        Raises:
            NotImplementedError: Thrown When the method was not overloaded by the task
                environment.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.

        Args:
            observations (np.ndarray): The observations.
            done (function): Whether the episode was done.

        Raises:
            NotImplementedError: Thrown When the method was not overloaded by the task
                environment.
        """
        raise NotImplementedError()
