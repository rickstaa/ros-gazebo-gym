#!/usr/bin/env python
import gym
from task_envs.task_envs_list import RegisterOpenAI_Ros_Env

def StartOpenAI_ROS_Environment(task_and_robot_environment_name):
    """
    It Does all the stuff that the user would have to do to make it simpler
    for the user.
    This means:
    0) Registers the TaskEnvironment wanted, if it exists in the Task_Envs.
    1) Loads the PARAMS needed for this Task.
    2) Checks that the workspace of the user has all that is needed for launching this.
    Which means that it will check that the robot spawn launch is there and the worls spawn is there.
    4) Launches the world launch and the robot spawn.
    5) It will import the Gym Env and Make it.
    """
    
    RegisterOpenAI_Ros_Env( task_env = task_and_robot_environment_name,
                            timestep_limit_per_episode = 10000)
    
    env = gym.make(task_and_robot_environment_name)
    