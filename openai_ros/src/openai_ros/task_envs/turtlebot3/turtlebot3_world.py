#!/usr/bin/env python3
"""Task environment for the Turtlebot3. This is the new version of the classic
Turtlebot2 created by `ROBOTIS <www.robotis.us>`_. The goal of the task environment is
for the agent to learn to move in the world without bumping into walls or objects.
"""
import os

import numpy as np
import rospy
from gym import spaces
from openai_ros.core import ROSLauncher
from openai_ros.core.helpers import (get_vector_magnitude,
                                     load_ros_params_from_yaml)
from openai_ros.robot_envs import turtlebot3_env


class TurtleBot3WorldEnv(turtlebot3_env.TurtleBot3Env):
    """Task Env designed for having the TurtleBot3 in the turtlebot3 world closed
    room with columns. It will learn how to move around without crashing.

    Attributes:
        linear_forward_speed (float): The linear forward speed of the robot.
        linear_turn_speed (float): The linear turn speed of the robot.
        angular_speed (float): The angular speed of the robot.
        init_linear_forward_speed (float): The initial linear forward speed of the
            robot.
        init_linear_turn_speed (float): The initial linear turn speed of the robot.
        new_ranges (int): In how many sections we want to defined the 360 degrees laser
            reading.
        min_range (float): Minimum meters below which we consider we have crashed into
            a object.
        max_laser_value (float): The maximum laser sensor value.
        min_laser_value (float): The minimum laser sensor value.
        max_linear_acceleration (float): The maximum linear acceleration.
        action_space (:obj:`gym.spaces.Box`): The robot action space.
        reward_range (tuple): The range of the reward.
        observation_space (:obj:`gym.spaces.Box`): The robot observation space
            (down-sampled and discretized laser scanner data).
        forwards_reward (float): The reward for going forward.
        turn_reward (float): The reward for turning.
        end_episode_points (float): The reward at the end of a episode.
        cumulated_steps (int): The number of steps that were taken.
    """

    def __init__(self):
        """Initialize turtlebot3 task environment instance."""
        # This is the path of the workspace where the simulation files can be found
        workspace_path = rospy.get_param("/turtlebot3/workspace_path", None)
        if workspace_path:
            assert os.path.exists(workspace_path), (
                "The Simulation ROS Workspace path "
                + workspace_path
                + " DOESN'T exist, execute: mkdir -p "
                + workspace_path
                + "/src;cd "
                + workspace_path
                + ";catkin_make"
            )

        # Launch the turtlebot3 gazebo environment
        # NOTE: This downloads and builds the required ROS packages if not found
        ROSLauncher.launch(
            package_name="turtlebot3_gazebo",
            launch_file_name="start_world.launch",
            workspace_path=workspace_path,
        )

        # Retrieve env params from the desired Yaml file
        load_ros_params_from_yaml(
            package_name="openai_ros",
            rel_path_from_package_to_file="src/openai_ros/task_envs/turtlebot3/config",
            yaml_file_name="turtlebot3_world.yaml",
        )
        self._get_params()

        # Initialize robot environment
        super(TurtleBot3WorldEnv, self).__init__(workspace_path)

        # Create action and reward space
        self.action_space = spaces.Discrete(self.n_actions)
        self.reward_range = (-np.inf, np.inf)
        rospy.logdebug("ACTION SPACES TYPE===>" + str(self.action_space))

        # Create observation space
        # NOTE: We create two arrays based on the binary values that will be assigned
        # In the discrimination method.
        laser_scan = self.get_laser_scan()
        num_laser_readings = int(len(laser_scan.ranges) / self.new_ranges)
        high = np.full((num_laser_readings), self.max_laser_value)
        low = np.full((num_laser_readings), self.min_laser_value)
        self.laser_space = spaces.Box(low, high)
        obs_high = np.full((self.new_ranges), self.max_laser_value)
        obs_low = np.full((self.new_ranges), self.min_laser_value)
        self.observation_space = spaces.Box(obs_low, obs_high)
        rospy.logdebug("LASER OBSERVATION SPACES TYPE===>" + str(self.laser_space))
        rospy.logdebug(
            "DISCRETIZED OBSERVATION SPACES TYPE===>" + str(self.observation_space)
        )

        # Other variables
        self.cumulated_steps = 0.0

    #############################################
    # Task environment internal methods #########
    #############################################
    # NOTE: Here you can add additional helper methods that are used in the task env
    def _discretize_scan_observation(self, data, new_ranges):
        """Discretize the laser scan observations into a integer value.

        .. note::
            Done by downsizing the 360 laser scan data into a smaller amount of
            sections. Following we discard all the laser readings that are not multiple
            in index of new_rangesvalue.

        Args:
            data (:obj:`sensor_msgs.msg._LaserScan.LaserScan`): The laser scan data.
            new_ranges (int): The number of ranges (sections) you want to use for
                the downsampling.
        Returns:
            list: The discritezed down-sampled values.
        """
        self._episode_done = False
        discretized_ranges = []
        mod = len(data.ranges) / new_ranges
        rospy.logdebug("data=" + str(data))
        rospy.logdebug("new_ranges=" + str(new_ranges))
        rospy.logdebug("mod=" + str(mod))

        # Downsample range and discretize values
        for i, item in enumerate(data.ranges):
            if i % mod == 0:
                if item == float("Inf") or np.isinf(item):
                    discretized_ranges.append(self.max_laser_value)
                elif np.isnan(item):
                    discretized_ranges.append(self.min_laser_value)
                else:
                    discretized_ranges.append(int(item))
                if self.min_range > item > 0:
                    rospy.logerr(
                        "done Validation >>> item="
                        + str(item)
                        + "< "
                        + str(self.min_range)
                    )
                    self._episode_done = True
                else:
                    rospy.logdebug(
                        "NOT done Validation >>> item="
                        + str(item)
                        + "< "
                        + str(self.min_range)
                    )
        return discretized_ranges

    def _get_params(self):
        """Retrieve task environment configuration parameters from the parameter server."""
        self.n_actions = rospy.get_param("/turtlebot3/n_actions")
        self.linear_forward_speed = rospy.get_param("/turtlebot3/linear_forward_speed")
        self.linear_turn_speed = rospy.get_param("/turtlebot3/linear_turn_speed")
        self.angular_speed = rospy.get_param("/turtlebot3/angular_speed")
        self.init_linear_forward_speed = rospy.get_param(
            "/turtlebot3/init_linear_forward_speed"
        )
        self.init_linear_turn_speed = rospy.get_param(
            "/turtlebot3/init_linear_turn_speed"
        )
        self.new_ranges = rospy.get_param("/turtlebot3/new_ranges")
        self.min_range = rospy.get_param("/turtlebot3/min_range")
        self.max_laser_value = rospy.get_param("/turtlebot3/max_laser_value")
        self.min_laser_value = rospy.get_param("/turtlebot3/min_laser_value")
        self.max_linear_acceleration = rospy.get_param(
            "/turtlebot3/max_linear_acceleration"
        )
        self.forwards_reward = rospy.get_param("/turtlebot3/forwards_reward")
        self.turn_reward = rospy.get_param("/turtlebot3/turn_reward")
        self.end_episode_points = rospy.get_param("/turtlebot3/end_episode_points")

    #############################################
    # Overload Robot env virtual methods ########
    #############################################
    # NOTE: Methods that need to be implemented as they are called by the robot and
    # gazebo environments.
    def _set_init_pose(self):
        """Sets the Robot in its init pose."""
        self.move_base(
            self.init_linear_forward_speed,
            self.init_linear_turn_speed,
            epsilon=0.05,
            update_rate=10,
        )

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        self.cumulated_reward = 0.0  # For Info Purposes
        self._episode_done = (
            False  # Set to false Done, because its calculated asynchronously
        )

    def _set_action(self, action):
        """Applies the given action to the simulation.

        Args:
            action (numpy.ndarray): The action we want to apply.
        """
        rospy.logdebug("Start Set Action ==>" + str(action))

        # Convert the actions to speed movements
        if action == 0:  # FORWARD
            linear_speed = self.linear_forward_speed
            angular_speed = 0.0
            self.last_action = "FORWARDS"
        elif action == 1:  # LEFT
            linear_speed = self.linear_turn_speed
            angular_speed = self.angular_speed
            self.last_action = "TURN_LEFT"
        elif action == 2:  # RIGHT
            linear_speed = self.linear_turn_speed
            angular_speed = -1 * self.angular_speed
            self.last_action = "TURN_RIGHT"

        # We tell TurtleBot3 the linear and angular speed to set to execute
        self.move_base(linear_speed, angular_speed, epsilon=0.05, update_rate=10)
        rospy.logdebug("END Set Action ==>" + str(action))

    def _get_obs(self):
        """Here we define what sensor data of our robots observations we have access to.

        Returns:
            numpy.ndarray: The discretized observation data.
        """
        rospy.logdebug("Start Get Observation ==>")

        # We get the laser scan data
        laser_scan = self.get_laser_scan()
        discretized_observations = self._discretize_scan_observation(
            laser_scan, self.new_ranges
        )
        rospy.logdebug("Observations==>" + str(discretized_observations))
        rospy.logdebug("END Get Observation ==>")
        return discretized_observations

    def _is_done(self, observations):
        """Indicates whether or not the episode is done (the robot has fallen for
        example).

        Args:
            observations (numpy.ndarray): The observation vector.

        Returns:
            bool: Whether the episode was finished.
        """
        if self._episode_done:
            rospy.logerr("TurtleBot3 is Too Close to wall==>")
        else:
            rospy.logwarn("TurtleBot3 is NOT close to a wall ==>")

        # Now we check if it has crashed based on the imu
        imu_data = self.get_imu()
        linear_acceleration_magnitude = get_vector_magnitude(
            imu_data.linear_acceleration
        )
        if linear_acceleration_magnitude > self.max_linear_acceleration:
            rospy.logerr(
                "TurtleBot3 Crashed==>"
                + str(linear_acceleration_magnitude)
                + ">"
                + str(self.max_linear_acceleration)
            )
            self._episode_done = True
        else:
            rospy.logerr(
                "DIDN'T crash TurtleBot3 ==>"
                + str(linear_acceleration_magnitude)
                + ">"
                + str(self.max_linear_acceleration)
            )
        return self._episode_done

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.

        Args:
            observations (numpy.ndarray): The observation vector.
            done (bool): Whether the episode has finished.

        Returns:
            float: The step reward.
        """
        if not done:
            if self.last_action == "FORWARDS":
                reward = self.forwards_reward
            else:
                reward = self.turn_reward
        else:
            reward = -1 * self.end_episode_points
        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))
        return reward
