"""Contains functions and classes that are used to setup the openai_ros gym
environments. This contains functions for download the required dependencies, starting
ROS launch files, connecting to the gazebo simulation, ect.
"""
from openai_ros.core.start_openai_ros_env import start_openai_ros_env
from openai_ros.core.gazebo_connection import GazeboConnection
from openai_ros.core.ros_launcher import ROSLauncher
