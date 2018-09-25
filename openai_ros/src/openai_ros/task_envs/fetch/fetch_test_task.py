from gym import utils

import rospy
from gym import spaces
from openai_ros.robot_envs import fetch_env
from gym.envs.registration import register
import numpy as np
from sensor_msgs.msg import JointState


register(
        id='FetchTest-v0',
        entry_point='openai_ros:task_envs.fetch.fetch_test_task.FetchTestEnv',
        timestep_limit=1000,
    )
    

class FetchTestEnv(fetch_env.FetchEnv, utils.EzPickle):
    def __init__(self):
        
        rospy.logdebug("Entered FetchTestEnv Env")
        self.get_params()
        
        fetch_env.FetchEnv.__init__(self)

        
        self.action_space = spaces.Discrete(self.n_actions)
        
        observations_high_range = np.array([self.observations_max]*self.n_obervations)
        observations_low_range = np.array([self.observations_min]*self.n_obervations)
        self.observation_space = spaces.Box(observations_low_range, observations_high_range)
        

        
    def get_params(self):
        #get configuration parameters
        
        self.n_actions = rospy.get_param('/fetch/n_actions')
        self.n_obervations = rospy.get_param('/fetch/n_obervations')
        self.observations_max = rospy.get_param('/fetch/observations_max')
        self.observations_min = rospy.get_param('/fetch/observations_min')
        self.init_pos = rospy.get_param('/fetch/init_pos')
        self.gripper_extra_height = rospy.get_param('/fetch/gripper_extra_height')
        
        
    def _env_setup(self, initial_qpos):
        rospy.logdebug("Init Pos:")
        rospy.logdebug(initial_qpos)
        
        self.gazebo.unpauseSim()
        self.set_trajectory_joints(initial_qpos)

        # Move end effector into position.
        rospy.logerr("Moving To SETUP Position 1")
        gripper_target = np.array([0.498, 0.005, 0.431 + self.gripper_extra_height])
        gripper_rotation = np.array([1., 0., 1., 0.])
        action = np.concatenate([gripper_target, gripper_rotation])
        self.set_trajectory_ee(action)
        
        # For testing purposes
        rospy.logerr("Moving To SETUP Position 2")
        gripper_target = np.array([0.798, 0.005, 0.431 + self.gripper_extra_height])
        gripper_rotation = np.array([1., 0., 1., 0.])
        action = np.concatenate([gripper_target, gripper_rotation])
        self.set_trajectory_ee(action)
        


    def _set_action(self, action):
        # TODO: Make something usefull
        
        gripper_target = np.array([0.498, 0.005, 0.431 + self.gripper_extra_height*action])
        gripper_rotation = np.array([1., 0., 1., 0.])
        
        action = np.concatenate([gripper_target, gripper_rotation])
        # Apply action to simulation.
        self.set_trajectory_ee(action)

    def _get_obs(self):
        
        grip_pos = self.get_ee_pose()
        grip_pos_array = np.array([grip_pos.pose.position.x, grip_pos.pose.position.y, grip_pos.pose.position.z])
        grip_rpy = self.get_ee_rpy()
        grip_velp = np.array([grip_rpy.y, grip_rpy.y])
        robot_qpos, robot_qvel = self.robot_get_obs(self.joints)

        object_pos = object_rot = object_velp = object_velr = object_rel_pos = np.zeros(0)
            
        gripper_state = robot_qpos[-2:]
        gripper_vel = robot_qvel[-2:] #* dt  # change to a scalar if the gripper is made symmetric
   
        achieved_goal = self._sample_achieved_goal(grip_pos_array, object_pos)
        
        """    
        obs = np.concatenate([
            grip_pos_array, object_pos.ravel(), object_rel_pos.ravel(), gripper_state, object_rot.ravel(),
            object_velp.ravel(), object_velr.ravel(), gripper_vel,
        ])
        """
        # TODO: Make something usefull
        obs = [0]

        return obs
        
    def _is_done(self, observations):
        
        """
        d = self.goal_distance(observations['achieved_goal'], self.goal)
        return (d < self.distance_threshold).astype(np.float32)
        """
        # TODO: Make something usefull
        return observations[0] == 0
        
    def _compute_reward(self, observations, done):

        """
        d = self.goal_distance(observations['achieved_goal'], self.goal)
        if self.reward_type == 'sparse':
            return -(d > self.distance_threshold).astype(np.float32)
        else:
            return -d
        """
        # TODO: Make something usefull
        return observations[0]
        
    def _init_env_variables(self):
        """
        Inits variables needed to be initialized each time we reset at the start
        of an episode.
        :return:
        """
        pass
    
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.gazebo.unpauseSim()
        self.set_trajectory_joints(self.init_pos)

        return True
        
    def goal_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)
        

    def _sample_goal(self):

        goal = self.initial_gripper_xpos[:3] + self.np_random.uniform(-0.15, 0.15, size=3)
        return goal
        
    def _sample_achieved_goal(self, grip_pos_array, object_pos):
        if not self.has_object:
            achieved_goal = grip_pos_array.copy()
        else:
            achieved_goal = np.squeeze(object_pos.copy())
        
        #return achieved_goal.copy()
        return achieved_goal
        
    
    def robot_get_obs(data):
        
        """
        Returns all joint positions and velocities associated with a robot.
        """
    
        if data.position is not None and data.name:
            #names = [n for n in data.name if n.startswith('robot')]
            names = [n for n in data.name]
            i = 0
            r = 0
            for name in names:
                r += 1
                
            return (
                np.array([data.position[i] for i in range(r)]),
                np.array([data.velocity[i] for i in range(r)]),
            )
        return np.zeros(0), np.zeros(0)