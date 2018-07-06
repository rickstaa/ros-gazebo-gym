#!/usr/bin/env python

import gym
import rospy
import roslaunch
import time
import numpy as np
from gym import utils, spaces
#from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from gym.utils import seeding
from gym.envs.registration import register
import copy
import math
import os

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.msg import LinkState
from rosgraph_msgs.msg import Clock
from openai_ros import robot_gazebo_env


#class CartPoleEnv(gazebo_env.GazeboEnv):
class CartPoleEnv(robot_gazebo_env.RobotGazeboEnv):
    def __init__(
        self, control_type
    ):
        
        self.publishers_array = []
        self._base_pub = rospy.Publisher('/cartpole_v0/foot_joint_velocity_controller/command', Float64, queue_size=1)
        self._pole_pub = rospy.Publisher('/cartpole_v0/pole_joint_velocity_controller/command', Float64, queue_size=1)
        self.publishers_array.append(self._base_pub)
        self.publishers_array.append(self._pole_pub)
        
        rospy.Subscriber("/cartpole_v0/joint_states", JointState, self.joints_callback)
        
        self.control_type = control_type
        if self.control_type == "velocity":
            self.controllers_list = ['joint_state_controller',
                                    'pole_joint_velocity_controller',
                                    'foot_joint_velocity_controller',
                                    ]
                                    
        elif self.control_type == "position":
            self.controllers_list = ['joint_state_controller',
                                    'pole_joint_position_controller',
                                    'foot_joint_position_controller',
                                    ]
                                    
        elif self.control_type == "effort":
            self.controllers_list = ['joint_state_controller',
                                    'pole_joint_effort_controller',
                                    'foot_joint_effort_controller',
                                    ]

        self.robot_name_space = "cartpole_v0"
        self.reset_controls = True

        # Seed the environment
        self._seed()
        self.steps_beyond_done = None
        
        super(CartPoleEnv, self).__init__(
            controllers_list=self.controllers_list,
            robot_name_space=self.robot_name_space,
            reset_controls=self.reset_controls
            )

    def joints_callback(self, data):
        self.joints = data

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
        
    # RobotEnv methods
    # ----------------------------

    def _step_callback(self):
        #if self.block_gripper:
            #self.sim.data.set_joint_qpos('robot0:l_gripper_finger_joint', 0.)
            #self.sim.data.set_joint_qpos('robot0:r_gripper_finger_joint', 0.)
            #self.sim.forward()
        #self.execute_trajectory()
        pass

    def _viewer_setup(self):
        """
        body_id = self.sim.model.body_name2id('robot0:gripper_link')
        lookat = self.sim.data.body_xpos[body_id]
        for idx, value in enumerate(lookat):
            self.viewer.cam.lookat[idx] = value
        self.viewer.cam.distance = 2.5
        self.viewer.cam.azimuth = 132.
        self.viewer.cam.elevation = -14.
        """
        pass
        
    def _render_callback(self):
        """
        # Visualize target.
        sites_offset = (self.sim.data.site_xpos - self.sim.model.site_pos).copy()
        site_id = self.sim.model.site_name2id('target0')
        self.sim.model.site_pos[site_id] = self.goal - sites_offset[0]
        self.sim.forward()
        """
        pass
    
    """
    def _reset_sim(self):
        
        print "Started reset_sim"
        
        #self.pause()
        #self.resetSim()
        #self.unpause()
        rospy.logdebug("We UNPause the simulation to start having topic data")
        self.gazebo.unpauseSim()

        rospy.logdebug("CLOCK BEFORE RESET")
        self.get_clock_time()

        rospy.logdebug("SETTING INITIAL POSE TO AVOID")
        self.set_init_pose()
        time.sleep(self.wait_time * 2.0)
        rospy.logdebug("AFTER INITPOSE CHECKING SENSOR DATA")
        self.check_all_systems_ready()
        #rospy.logdebug("We deactivate gravity to check any reasidual effect of reseting the simulation")
        #self.gazebo.change_gravity(0.0, 0.0, 0.0)

        rospy.logdebug("RESETING SIMULATION")
        self.gazebo.pauseSim()
        self.gazebo.resetSim()
        self.gazebo.unpauseSim()
        rospy.logdebug("CLOCK AFTER RESET")
        self.get_clock_time()

        rospy.logdebug("RESETING CONTROLLERS SO THAT IT DOESNT WAIT FOR THE CLOCK")
        self.controllers_object.reset_cartpole_joint_controllers()
        rospy.logdebug("AFTER RESET CHECKING SENSOR DATA")
        self.check_all_systems_ready()
        rospy.logdebug("CLOCK AFTER SENSORS WORKING AGAIN")
        self.get_clock_time()
        #rospy.logdebug("We reactivating gravity...")
        #self.gazebo.change_gravity(0.0, 0.0, -9.81)
        rospy.logdebug("END")

        # 7th: pauses simulation
        rospy.logdebug("Pause SIM...")
        self.gazebo.pauseSim()
        print "Finish reset_sim"

        # get the last observation got when paused, generated by the callbakc or the check_all_systems_ready
        # Depends on who was last
        #observation, _, state = self.observation_checks()

        #return observation
        return True
    """    
        
    """
    def _sample_goal(self):
        if self.has_object:
            goal = self.initial_gripper_xpos[:3] + self.np_random.uniform(-self.target_range, self.target_range, size=3)
            goal += self.target_offset
            goal[2] = self.height_offset
            if self.target_in_the_air and self.np_random.uniform() < 0.5:
                goal[2] += self.np_random.uniform(0, 0.45)
        else:
            goal = self.initial_gripper_xpos[:3] + self.np_random.uniform(-0.15, 0.15, size=3)
        return goal.copy()
    """

    def _env_setup(self, initial_qpos):
        self.init_internal_vars(self.init_pos)
        self.set_init_pose()
        self.check_all_systems_ready()
        
    def init_internal_vars(self, init_pos_value):
        self.pos = [init_pos_value]
        self.joints = None
        
    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while (self._base_pub.get_num_connections() == 0 and not rospy.is_shutdown()):
            rospy.logdebug("No susbribers to _base_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_base_pub Publisher Connected")

        while (self._pole_pub.get_num_connections() == 0 and not rospy.is_shutdown()):
            rospy.logdebug("No susbribers to _pole_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_pole_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def _check_all_systems_ready(self, init=True):
        self.base_position = None
        while self.base_position is None and not rospy.is_shutdown():
            try:
                self.base_position = rospy.wait_for_message("/cartpole_v0/joint_states", JointState, timeout=1.0)
                rospy.logdebug("Current cartpole_v0/joint_states READY=>"+str(self.base_position))
                if init:
                    # We Check all the sensors are in their initial values
                    positions_ok = all(abs(i) <= 1.0e-02 for i in self.base_position.position)
                    velocity_ok = all(abs(i) <= 1.0e-02 for i in self.base_position.velocity)
                    efforts_ok = all(abs(i) <= 1.0e-01 for i in self.base_position.effort)
                    base_data_ok = positions_ok and velocity_ok and efforts_ok
                    rospy.logdebug("Checking Init Values Ok=>" + str(base_data_ok))
            except:
                rospy.logerr("Current cartpole_v0/joint_states not ready yet, retrying for getting joint_states")
        rospy.logdebug("ALL SYSTEMS READY")
        
            
    def move_joints(self, joints_array):
        joint_value = Float64()
        joint_value.data = joints_array[0]
        rospy.logdebug("Single Base JointsPos>>"+str(joint_value))
        self._base_pub.publish(joint_value)


    def _set_init_pose(self):
        """
        Sets joints to initial position [0,0,0]
        :return:
        """
        
        self.check_publishers_connection()
        
        # Reset Internal pos variable
        self.init_internal_vars(self.init_pos)
        self.move_joints(self.pos)
        
    def get_clock_time(self):
        self.clock_time = None
        while self.clock_time is None and not rospy.is_shutdown():
            try:
                self.clock_time = rospy.wait_for_message("/clock", Clock, timeout=1.0)
                rospy.logdebug("Current clock_time READY=>" + str(self.clock_time))
            except:
                rospy.logdebug("Current clock_time not ready yet, retrying for getting Current clock_time")
        return self.clock_time
    