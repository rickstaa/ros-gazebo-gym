#!/usr/bin/env python
import gym
from task_envs.task_envs_list import RegisterOpenAI_Ros_Env
import roslaunch
import rospy
import rospkg
import os

def StartOpenAI_ROS_Environment(task_and_robot_environment_name):
    """
    It Does all the stuff that the user would have to do to make it simpler
    for the user.
    This means:
    0) Registers the TaskEnvironment wanted, if it exists in the Task_Envs.
    2) Checks that the workspace of the user has all that is needed for launching this.
    Which means that it will check that the robot spawn launch is there and the worls spawn is there.
    4) Launches the world launch and the robot spawn.
    5) It will import the Gym Env and Make it.
    """
    
    result = RegisterOpenAI_Ros_Env( task_env = task_and_robot_environment_name,
                            timestep_limit_per_episode = 10000)
    
    
    if result:
        print("Register of Task Env went OK, lets make the env...")
        env = gym.make(task_and_robot_environment_name)
    else:
        print("Something Went wrong in the register")
        env = None
        
    return env
    
    
    
class ROSLauncher(object):
    def __init__(self, rospackage_name, launch_file_name, ros_ws_abspath="/home/user/simulation_ws"):

        self._rospackage_name = rospackage_name
        self._launch_file_name = launch_file_name
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(rospackage_name)
        
        # Check Package Exists
        try:
            pkg_path = rospack.get_path(rospackage_name)
        except rospkg.common.ResourceNotFound:
            print("Package is not found")
            pkg_path = None
            
            DownloadRepo(   package_name=rospackage_name,
                            ros_ws_abspath=ros_ws_abspath)
            rospy.logwarn("Please run catkin_make in the ros_ws="+ros_ws_abspath+", source devel/setup.bash and rospack profile before running again.")
            
        
        # If the package was found then we launch
        if pkg_path:
            rospy.loginfo(">>>>>>>>>>Package found in workspace-->"+str(pkg_path))
            launch_dir = os.path.join(pkg_path, "launch") 
            path_launch_file_name = os.path.join(launch_dir, launch_file_name)
            
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [path_launch_file_name])
            self.launch.start()
            
            rospy.loginfo("STARTED Roslaunch-->"+str(self._launch_file_name))
        
        
        
    def DownloadRepo(self, package_name, ros_ws_abspath):
        """
        This has to be installed
        sudo pip install gitpython
        """
    
        ros_ws_src_abspath_src = os.path.join(ros_ws_abspath,"src")
    
        # We retrieve the got for the package asked
        package_git = None
        if package_name == "moving_cube_description":
            package_git = "git clone https://bitbucket.org/theconstructcore/moving_cube.git"
        else:
            rospy.logerr("Package [ "+package_name+" ] is not supported for autodownload")
            assert False, "The package "++" is not supported, please check the package name and the git support in openai_ros_common.py"
        
        # If a Git for the package is supported
        if package_git:
            try:
                git.Git(ros_ws_src_abspath_src).clone(package_git)
                # We have to make the user compile and source to make ROS be able to find the new packages
                # TODO: Make this automatic
                assert False, "You need to compile, source and rospack profile and rerun to dowloads to take effect."
            except git.exc.GitCommandError:
                rospy.logwarn("The Git already exists in that path, not downloading")
    
            
            

   
    