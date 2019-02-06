#!/usr/bin/env python
import gym
from task_envs.task_envs_list import RegisterOpenAI_Ros_Env
import roslaunch
import rospy
import rospkg

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
    
    RegisterOpenAI_Ros_Env( task_env = task_and_robot_environment_name,
                            timestep_limit_per_episode = 10000)
    
    env = gym.make(task_and_robot_environment_name)
    
class ROSLauncher(object):
    def __init__(self, rospackage_name, launch_file_name):

        self._rospackage_name = rospackage_name
        self._launch_file_name = launch_file_name
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(rospackage_name)
        launch_dir = os.path.join(pkg_path, "launch") 
        path_launch_file_name = os.path.join(launch_dir, launch_file_name)
        
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [path_launch_file_name])
        self.launch.start()
        
        rospy.loginfo("STARTED Roslaunch-->"+str(self._launch_file_name))
        
        
    def __del__(self):
        print("Shutting down the roslaunch--->"+str(self._launch_file_name))
        self.launch.shutdown()
   
    