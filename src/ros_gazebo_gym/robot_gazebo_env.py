"""The Gazebo environment is mainly used to connect the simulated Gym environment to the
Gazebo simulator. It takes care of the resets of the simulator after each step or the
resets of the controllers (if needed), it also takes care of all the steps that need to
be done on the simulator when doing a training step or a training reset (typical steps
in the reinforcement learning loop).
"""

import gymnasium as gym
import rospy
from ros_gazebo_gym.common.markers.text_overlay import TextOverlay
from ros_gazebo_gym.core.controllers_connection import ControllersConnection
from ros_gazebo_gym.core.gazebo_connection import GazeboConnection
from ros_gazebo_gym.core.helpers import ros_exit_gracefully
from ros_gazebo_gym.msg import RLExperimentInfo


class RobotGazeboEnv(gym.Env):
    """Connects the simulated environment to the gazebo simulator.

    Attributes:
        gazebo (:class:`~ros_gazebo_gym.core.gazebo_connection.GazeboConnection`):
            Gazebo connector which can be used to interact with the gazebo simulation.
        episode_num (int): The current episode.
        step_num (int): The current step.
        cumulated_episode_reward (float): The cumulated episode reward.
    """

    def __init__(
        self,
        robot_name_space,
        reset_controls,
        controllers_list=None,
        reset_robot_pose=False,
        reset_world_or_sim="SIMULATION",
        log_reset=True,
        pause_simulation=False,
        publish_rviz_training_info_overlay=False,
    ):
        """Initiate the RobotGazebo environment instance.

        Args:
            robot_name_space (str): The namespace the robot is on.
            reset_controls (bool): Whether the controllers should be reset when the
                :meth:`RobotGazeboEnv.reset` method is called.
            controllers_list (list, optional): A list with currently available
                controllers to look for. Defaults to ``None``, which means that the
                class will try to retrieve all the running controllers.
            reset_robot_pose (bool): Boolean specifying whether to reset the robot pose
                when the simulation is reset.
            reset_world_or_sim (str, optional): Whether you want to reset the whole
                simulation "SIMULATION" at startup or only the world "WORLD" (object
                positions). Defaults to "SIMULATION".
            log_reset (bool, optional): Whether we want to print a log statement when
                the world/simulation is reset. Defaults to ``True``.
            pause_sim (bool, optional): Whether the simulation should be paused after
                each step (i.e. after each action). Defaults to ``False``.
            publish_rviz_training_info_overlay (bool, optional): Whether a RViz overlay
                should be published with the training results. Defaults to ``False``.
        """
        rospy.logdebug("START init RobotGazeboEnv")
        self.gazebo = GazeboConnection(reset_world_or_sim, log_reset=log_reset)
        self._controllers_object = ControllersConnection(
            namespace=robot_name_space, controllers_list=controllers_list
        )
        self._reset_controls = reset_controls
        self._reset_robot_pose = reset_robot_pose
        self._pause_simulation = pause_simulation
        self._publish_rviz_training_info_overlay = publish_rviz_training_info_overlay

        # Set up ROS related variables.
        self.episode_num = 0
        self.step_num = 0
        self.cumulated_episode_reward = 0

        # Create training info publishers.
        self._reward_pub = rospy.Publisher(
            "/ros_gazebo_gym/reward", RLExperimentInfo, queue_size=1, latch=True
        )
        if self._publish_rviz_training_info_overlay:
            self._rviz_training_info_overlay_pub = rospy.Publisher(
                "/ros_gazebo_gym/rviz_training_info_overlay",
                TextOverlay,
                queue_size=1,
                latch=True,
            )

        # Set physics engine properties when the are specified by the user.
        try:
            self._set_init_gazebo_variables()
        except NotImplementedError:
            pass

        # Un-pause the simulation and reset the controllers if needed.
        """To check any topic we need to have the simulations running, we need to do
        two things:
            1) Un-pause the simulation: without that the stream of data doesn't flow.
                This is for simulations that are pause for whatever the reason.
            2) If the simulation was running already for some reason, we need to reset
                the controllers. This has to do with the fact that some plugins with tf,
                don't understand the reset of the simulation and need to be reset to
                work properly.
        """
        if self._reset_controls:
            self.gazebo.pause_sim()  # Done to prevent robot movement.
            self._controllers_object.reset_controllers()
        self.gazebo.unpause_sim()
        rospy.logdebug("END init RobotGazeboEnv")

    ################################################
    # Main environment methods #####################
    ################################################
    def step(self, action):
        """Function executed each time step. Here we get the action execute it in a
        time step and retrieve the observations generated by that action.

        Args:
            action (numpy.ndarray): The action we want to perform in the environment.

        Returns:
            (tuple): tuple containing:

                - obs (:obj:`np.ndarray`): Environment observation.
                - cost (:obj:`float`): Cost of the action.
                - terminated (:obj:`bool`): Whether the episode is terminated.
                - truncated (:obj:`bool`): Whether the episode was truncated. This
                  value is set by wrappers when for example a time limit is reached or
                  the agent goes out of bounds.
                - info (:obj:`dict`): Additional information about the environment.

        .. note::
            Here we should convert the action num to movement action, execute the action
            in the simulation and get the observations result of performing that action.
        """
        rospy.logdebug(f">> START STEP {self.step_num}")
        self.gazebo.unpause_sim()
        self._set_action(action)
        obs = self._get_obs()
        if self._pause_simulation:
            self.gazebo.pause_sim()
        done = self._is_done(obs)
        reward = self._compute_reward(obs, done)
        if self._publish_rviz_training_info_overlay:
            self._publish_rviz_info_overlay()
        self.cumulated_episode_reward += reward
        self.step_num += 1
        info = self._get_info()

        rospy.logdebug("END STEP")
        return obs, reward, done, False, info

    def _publish_reward_topic(self):
        """This function publishes the given reward in the reward topic for
        easy access from ROS infrastructure.
        """
        reward_msg = RLExperimentInfo()
        reward_msg.episode_number = self.episode_num
        reward_msg.reward = self.cumulated_episode_reward
        self._reward_pub.publish(reward_msg)

    def _publish_rviz_info_overlay(self):
        """Publishes the RViz training text overlay."""
        self._rviz_training_info_overlay_pub.publish(
            TextOverlay(
                text=(
                    f"Cumulated Reward: {self.cumulated_episode_reward}\n"
                    f"Episode: {self.episode_num}\nStep: {self.step_num}"
                )
            )
        )

    def _update_episode(self):
        """Publishes the cumulated reward of the episode and
        increases the episode number by one.
        """
        rospy.logwarn(f"PUBLISHING EPISODE {self.episode_num} REWARD...")
        self._publish_reward_topic(self.cumulated_episode_reward, self.episode_num)
        rospy.logwarn(
            "PUBLISHING REWARD...DONE="
            + str(self.cumulated_episode_reward)
            + ",EP="
            + str(self.episode_num)
        )

        self.episode_num += 1
        self.step_num = 1
        self.cumulated_episode_reward = 0

    def reset(self, seed=None, options=None):
        """Function executed when resetting the environment.

        Args:
            seed (int, optional): The seed to use for the random number generator.
                Defaults to ``None``.
            options (dict, optional): The options to pass to the environment. Defaults
                to ``None``.

        Returns:
            (tuple): tuple containing:

                - obs (:obj:`numpy.ndarray`): The current state
                - info_dict (:obj:`dict`): Dictionary with additional information.
        """
        super().reset(seed=seed)

        rospy.logdebug("Resetting RobotGazeboEnvironment")
        self._reset_sim()
        self._init_env_variables()
        self._update_episode()
        obs = self._get_obs()
        if self._pause_simulation:
            self.gazebo.pause_sim()
        info = self._get_info()
        rospy.logdebug("END resetting RobotGazeboEnvironment")

        return obs, info

    def close(self):
        """Function executed when closing the environment. Use it for closing GUIS and
        other systems that need closing.
        """
        rospy.logdebug("Closing RobotGazeboEnvironment")
        ros_exit_gracefully(
            shutdown_msg=f"Shutting down {rospy.get_name()}", exit_code=0
        )

    def _reset_sim(self):
        """Resets a simulation."""
        # NOTE: Controllers are reset two times to make sure that control commands of
        # the previous episode are not applied to the new episode.
        rospy.logdebug("RESET SIM START")
        if self._reset_controls:
            rospy.logdebug("RESET CONTROLLERS")
            self.gazebo.unpause_sim()
            self._controllers_object.reset_controllers()
            self._check_all_systems_ready()
            end_joint_states = list(self.joint_states.position)
            self.gazebo.pause_sim()
            self.gazebo.reset_sim()
            self.gazebo.unpause_sim()
            # NOTE: The code below is needed since the reset behaviour differs between
            # physics engines (see https://github.com/osrf/gazebo/issues/3150).
            self.gazebo.set_model_configuration(
                model_name="panda",
                joint_names=self.joint_states.name,
                joint_positions=end_joint_states,
            )
            self._check_all_systems_ready()
            if self._reset_robot_pose:
                self._set_init_pose()
            self._controllers_object.reset_controllers()
            self._check_all_systems_ready()
        else:
            rospy.logwarn("DON'T RESET CONTROLLERS")
            self.gazebo.unpause_sim()
            self._check_all_systems_ready()
            end_joint_states = list(self.joint_states.position)
            self.gazebo.pause_sim()
            self.gazebo.reset_sim()
            self.gazebo.unpause_sim()
            # NOTE: The code below is needed since the reset behaviour differs between
            # physics engines (see https://github.com/osrf/gazebo/issues/3150).
            self.gazebo.set_model_configuration(
                model_name="panda",
                joint_names=self.joint_states.name,
                joint_positions=end_joint_states,
            )
            self._check_all_systems_ready()
            if self._reset_robot_pose:
                self._set_init_pose()
            self._check_all_systems_ready()

        rospy.logdebug("RESET SIM END")
        return True

    def render(self, render_mode="human"):
        """Overload render method since rendering is handled in Gazebo."""
        pass

    def pause_controllers(self, controllers_list=None, filter_list=[]):
        """Pauses the controllers.

        Args:
            controller_list (list, optional): The controllers you want to pause.
                Defaults to ``None``, which means that the class will pause all the
                running controllers.
            filter_list (list, optional): The controllers you want to ignore when
                pausing. Defaults to ``[]``.
        """
        self._controllers_object.pause_controllers(
            controller_list=controllers_list, filter_list=filter_list
        )

    def unpause_controllers(self):
        """Un-pauses the paused controllers."""
        self._controllers_object.unpause_controllers()

    ################################################
    # Extension methods ############################
    ################################################
    # NOTE: These methods CAN be overloaded by robot or task env)
    # - Task environment methods -
    def _set_init_gazebo_variables(self):
        """Initializes variables that need to be initialized at the start of the gazebo
        simulation. This function can for example be used to change the physics
        properties of the physics engine by using
        :obj:`~ros_gazebo_gym.core.gazebo_connection.set_physics_properties` method.

        .. note::
            This function is only run once when the :class:`RobotGazeboEnv` class
            is initialized. Please use the :meth:`RobotGazeboEnv,_init_env_variables`
            method if you need to initialize variables at the start of each episode.

        Raises:
            NotImplementedError: Thrown when not overloaded by the task environment.
        """
        raise NotImplementedError()

    def _set_init_pose(self):
        """Sets the Robot in its init pose.

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

    def _get_obs(self):
        """Returns the observation.

        Raises:
            NotImplementedError: Thrown when not overloaded by the task environment.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.

        Args:
            action (numpy.ndarray): The action you want to set.

        Raises:
            NotImplementedError: Thrown When the method was not overloaded by the task
                environment.
        """
        raise NotImplementedError()

    def _is_done(self, observations):
        """Indicates whether or not the episode is done (the robot has fallen for
        example).

        Args:
            observations (numpy.ndarray): The observations.

        Returns:
            bool: Whether the episode was finished.

        Raises:
            NotImplementedError: Thrown When the method was not overloaded by the task
                environment.
        """
        raise NotImplementedError()

    def _get_info(self):
        """Returns a dictionary with additional step information.

        Returns:
            dict: Dictionary with additional information.
        """
        return {}

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.

        Args:
            observations (numpy.ndarray): The observations.
            done (bool): Whether the episode was done.

        Raises:
            NotImplementedError: Thrown When the method was not overloaded by the task
                environment.
        """
        raise NotImplementedError()

    # - Robot environment methods -
    def _check_all_systems_ready(self):
        """Checks that all the sensors, publishers and other simulation systems are
        operational.

        Raises:
            NotImplementedError: Thrown when not overloaded by the robot environment.
        """
        raise NotImplementedError()
