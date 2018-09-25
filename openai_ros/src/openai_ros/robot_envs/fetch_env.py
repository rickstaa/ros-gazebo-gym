import numpy
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from openai_ros import robot_gazebo_env
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg


class FetchEnv(robot_gazebo_env.RobotGazeboEnv):

    def __init__(self):
        rospy.logdebug("Entered Fetch Env")
        # We Start all the ROS related Subscribers and publishers
        
        self.JOINT_STATES_SUBSCRIBER = '/joint_states'
        
        self._check_all_systems_ready()
        
        self.joint_states_sub = rospy.Subscriber(self.JOINT_STATES_SUBSCRIBER, JointState, self.joints_callback)
        self.joints = JointState()
        
        # Start Services
        self.move_fetch_object = MoveFetch()
        
        # Variables that we give through the constructor.

        self.controllers_list = []

        self.robot_name_space = ""
        self.reset_controls = True
        
        super(FetchEnv, self).__init__(
            controllers_list=self.controllers_list,
            robot_name_space=self.robot_name_space,
            reset_controls=self.reset_controls
            )



    # RobotGazeboEnv virtual methods
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True


    # FetchEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        self._check_joint_states_ready()
        
        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message(self.JOINT_STATES_SUBSCRIBER, JointState, timeout=1.0)
                rospy.logdebug("Current "+str(self.JOINT_STATES_SUBSCRIBER)+" READY=>" + str(self.joints))

            except:
                rospy.logerr("Current "+str(self.JOINT_STATES_SUBSCRIBER)+" not ready yet, retrying....")
        return self.joints
    
    def joints_callback(self, data):
        self.joints = data

    def get_joints(self):
        return self.joints

    def set_trajectory_ee(self, action):
        """
        Helper function.
        Wraps an action vector of joint angles into a JointTrajectory message.
        The velocities, accelerations, and effort do not control the arm motion
        """
        # Set up a trajectory message to publish.
        ee_target = geometry_msgs.msg.Pose()
        ee_target.pose.orientation.w = 1.0
        ee_target.pose.position.x = action[0]
        ee_target.pose.position.y = action[1]
        ee_target.pose.position.z = action[2]
        
        self.move_fetch_object.ee_traj(ee_target)
        
        return True
        
    def set_trajectory_joints(self, initial_qpos):
        """
        Helper function.
        Wraps an action vector of joint angles into a JointTrajectory message.
        The velocities, accelerations, and effort do not control the arm motion
        """
        # Set up a trajectory message to publish.
        
        joint_point = JointTrajRequest()
        
        positions_array = [None] * 7
        positions_array[0] = initial_qpos["joint0"]
        positions_array[1] = initial_qpos["joint1"]
        positions_array[2] = initial_qpos["joint2"]
        positions_array[3] = initial_qpos["joint3"]
        positions_array[4] = initial_qpos["joint4"]
        positions_array[5] = initial_qpos["joint5"]
        positions_array[6] = initial_qpos["joint6"]
 
        self.move_fetch_object.joint_traj(positions_array)
        
        return True
        
    def get_ee_pose(self):
        
        gripper_pose = self.move_fetch_object.ee_pose()
        
        return gripper_pose
        
    def get_ee_rpy(self):
        
        gripper_rpy = self.move_fetch_object.ee_rpy()
        
        return gripper_rpy
    
    # ParticularEnv methods
    # ----------------------------

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
        
        
class MoveFetch(object):
    
    def __init__(self):
        rospy.logdebug("In Move Fetch Calss init...")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.logdebug("moveit_commander initialised...")
        
        rospy.logdebug("Starting Robot Commander...")
        self.robot = moveit_commander.RobotCommander()
        rospy.logdebug("Starting Robot Commander...DONE")
        
        self.scene = moveit_commander.PlanningSceneInterface()  
        rospy.logdebug("PlanningSceneInterface initialised...DONE")
        self.group = moveit_commander.MoveGroupCommander("arm")
        rospy.logdebug("MoveGroupCommander for arm initialised...DONE")

        
    def ee_traj(self, pose):
        
        self.group.set_pose_target(pose)
        
        self.execute_trajectory()
        
        return True
        
    def joint_traj(self, positions_array):
        
        self.group_variable_values = self.group.get_current_joint_values()
        print ("Group Vars:")
        print (self.group_variable_values)
        print ("Point:")
        print (positions_array)
        self.group_variable_values[0] = positions_array[0]
        self.group_variable_values[1] = positions_array[1]
        self.group_variable_values[2] = positions_array[2]
        self.group_variable_values[3] = positions_array[3]
        self.group_variable_values[4] = positions_array[4]
        self.group_variable_values[5] = positions_array[5]
        self.group_variable_values[6] = positions_array[6]
        self.group.set_joint_value_target(self.group_variable_values)
        self.execute_trajectory()
        
        return True
        
    def execute_trajectory(self):
        
        self.plan = self.group.plan()
        self.group.go(wait=True)

    def ee_pose(self):
        
        gripper_pose = self.group.get_current_pose()

        return gripper_pose
        
    def ee_rpy(self, request):
        
        gripper_rpy = self.group.get_current_rpy()

        return gripper_rpy