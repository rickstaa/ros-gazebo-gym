"""Environment for the 3D CartPole stay up task. In this task environment the agent has
to learn to keep the pole upright. This task was based on the
CartPole-v1 environment found in the
`OpenAi gym package<https://gym.openai.com/envs/CartPole-v1/>`_.
"""

import os

import numpy as np
import rospy
from gym import spaces
from openai_ros.core import ROSLauncher
from openai_ros.core.helpers import load_ros_params_from_yaml
from openai_ros.robot_envs import cartpole_env


class CartPoleStayUpEnv(cartpole_env.CartPoleEnv):
    """3D task environment in which the agent has to keep the CartPole upright."""

    def __init__(self):
        """Initiate CartPoleStayUp task environment instance."""
        # This is the path where the simulation files, the Task and the Robot gits will
        # be downloaded if not there
        workspace_path = rospy.get_param("/cartpole_v0/workspace_path", None)
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
            package_name="cartpole_description",
            launch_file_name="start_world.launch",
            workspace_path=workspace_path,
        )

        # Retrieve env params from the desired Yaml file
        load_ros_params_from_yaml(
            package_name="openai_ros",
            rel_path_from_package_to_file="src/openai_ros/task_envs/cartpole_stay_up/config",  # noqa: E501
            yaml_file_name="stay_up.yaml",
        )
        self._get_params()

        # Initialize robot environment
        super(CartPoleStayUpEnv, self).__init__(
            control_type=self.control_type, workspace_path=workspace_path
        )

        # Create action and observation space
        self.action_space = spaces.Discrete(self.n_actions)
        high = np.array(
            [2.5 * 2, np.finfo(np.float32).max, 0.7 * 2, np.finfo(np.float32).max]
        )
        self.observation_space = spaces.Box(-high, high)

    #############################################
    # Task environment internal methods #########
    #############################################
    # NOTE: Here you can add additional helper methods that are used in the task env
    def _get_params(self):
        """Retrieve task environment configuration parameters from the parameter server."""
        self.n_actions = rospy.get_param("/cartpole_v0/n_actions")
        self.min_pole_angle = rospy.get_param("/cartpole_v0/min_pole_angle")
        self.max_pole_angle = rospy.get_param("/cartpole_v0/max_pole_angle")
        self.max_base_velocity = rospy.get_param("/cartpole_v0/max_base_velocity")
        self.min_base_pose_x = rospy.get_param("/cartpole_v0/min_base_pose_x")
        self.max_base_pose_x = rospy.get_param("/cartpole_v0/max_base_pose_x")
        self.pos_step = rospy.get_param("/cartpole_v0/pos_step")
        self.running_step = rospy.get_param("/cartpole_v0/running_step")
        self.init_pos = rospy.get_param("/cartpole_v0/init_pos")
        self.wait_time = rospy.get_param("/cartpole_v0/wait_time")
        self.control_type = rospy.get_param("/cartpole_v0/control_type")

    #############################################
    # Overload Robot env virtual methods ########
    #############################################
    # NOTE: Methods that need to be implemented as they are called by the robot and
    # gazebo environments.
    def _set_init_pose(self):
        """Sets joints to initial position (i.e. [0, 0, 0])."""
        self.pos = [self.init_pos]
        self.joints = None
        self.move_joints(self.pos)

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        self.steps_beyond_done = None

    def _set_action(self, action):
        """Applies the given action to the simulation.

        Args:
            action (numpy.ndarray): The action we want to apply.
        """
        # Take action
        if action == 0:  # LEFT
            rospy.loginfo("GO LEFT...")
            self.pos[0] -= self.pos_step
        elif action == 1:  # RIGHT
            rospy.loginfo("GO RIGHT...")
            self.pos[0] += self.pos_step
        elif action == 2:  # LEFT BIG
            rospy.loginfo("GO LEFT BIG...")
            self.pos[0] -= self.pos_step * 10
        elif action == 3:  # RIGHT BIG
            rospy.loginfo("GO RIGHT BIG...")
            self.pos[0] += self.pos_step * 10

        # Apply action to simulation.
        rospy.loginfo("MOVING TO POS==" + str(self.pos))
        self.move_joints(self.pos)
        rospy.logdebug(
            "Wait for some time to execute movement, time=" + str(self.running_step)
        )
        rospy.sleep(self.running_step)  # wait for some time
        rospy.logdebug(
            "DONE Wait for some time to execute movement, time="
            + str(self.running_step)
        )

    def _get_obs(self):
        """Here we define what sensor data of our robots observations we have access to.

        Returns:
            numpy.ndarray: The discretized observation data.
        """
        data = self.joints
        return np.array(
            [
                data.position[1],  # Base
                data.velocity[1],  # Base
                data.position[0],  # Pole
                data.velocity[0],  # Pole
            ]
        )

    def _is_done(self, observations):
        """Indicates whether or not the episode is done (the robot has fallen for
        example).

        Args:
            observations (numpy.ndarray): The observation vector.

        Returns:
            bool: Whether the episode was finished.
        """
        rospy.logdebug("START '_is_done'")
        done = False
        rospy.loginfo("BASEPOSITION==" + str(observations[0]))
        rospy.loginfo("POLE ANGLE==" + str(observations[2]))

        # Check if the base is still within the ranges of (-2, 2)
        if (
            self.min_base_pose_x >= observations[0]
            or observations[0] >= self.max_base_pose_x
        ):
            rospy.logerr(
                "Base Outside Limits==>min="
                + str(self.min_base_pose_x)
                + ",pos="
                + str(observations[0])
                + ",max="
                + str(self.max_base_pose_x)
            )
            done = True

        # Check if pole has toppled over
        if (
            self.min_pole_angle >= observations[2]
            or observations[2] >= self.max_pole_angle
        ):
            rospy.logerr(
                "Pole Angle Outside Limits==>min="
                + str(self.min_pole_angle)
                + ",pos="
                + str(observations[2])
                + ",max="
                + str(self.max_pole_angle)
            )
            done = True
        rospy.logdebug("FINISHED get '_is_done'")
        return done

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.

        Args:
            observations (numpy.ndarray): The observation vector.
            done (bool): Whether the episode has finished.

        Returns:
            float: The step reward.
        """
        rospy.logdebug("START _compute_reward")
        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None:
            # Pole just fell!
            self.steps_beyond_done = 0
            reward = 1.0
        else:
            if self.steps_beyond_done == 0:
                rospy.logwarn(
                    "You are calling 'step()' even though this environment has already "
                    "returned done = True. You should always call 'reset()' once you "
                    "receive 'done = True' -- any further steps are undefined behaviour"
                    "."
                )
            self.steps_beyond_done += 1
            reward = 0.0
        rospy.logdebug("END _compute_reward")
        return reward
