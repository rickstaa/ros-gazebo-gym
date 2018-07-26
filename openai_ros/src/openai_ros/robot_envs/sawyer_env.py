import numpy
import rospy
import time
from openai_ros import robot_gazebo_env
import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION



class SawyerEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all SawyerEnv environments.
    """

    def __init__(self):
        """
        Initializes a new SawyerEnv environment.
        
        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.
        
        The Sensors: The sensors accesible are the ones considered usefull for AI learning.
        
        Sensor Topic List:
        * /wamv/odom: Odometry of the Base of Wamv
        
        Actuators Topic List: 
        * As actuator we will use a class to interface with the movements through commands.
        
        Args:
        """
        rospy.logdebug("Start SawyerEnv INIT...")
        # Variables that we give through the constructor.
        # None in this case

        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = []

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(SawyerEnv, self).__init__(controllers_list=self.controllers_list,
                                            robot_name_space=self.robot_name_space,
                                            reset_controls=False,
                                            start_init_physics_parameters=False,
                                            reset_world_or_sim="WORLD")



        rospy.logdebug("SawyerEnv unpause1...")
        self.gazebo.unpauseSim()
        #self.controllers_object.reset_controllers()
        
        # TODO: Fill it with the sensors
        self._check_all_systems_ready()

        self._check_movement_system_ready()

        self.gazebo.pauseSim()
        
        rospy.logdebug("Finished SawyerEnv INIT...")

    # Methods needed by the RobotGazeboEnv
    # ----------------------------
    

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        rospy.logdebug("SawyerEnv check_all_systems_ready...")
        self._check_all_sensors_ready()
        rospy.logdebug("END SawyerEnv _check_all_systems_ready...")
        return True


    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        # TODO: Here go the sensors like cameras and joint states
        rospy.logdebug("ALL SENSORS READY")

        
    def _check_movement_system_ready(self):
        """
        Setup of the movement system.
        :return:
        """
        rp = intera_interface.RobotParams()
        valid_limbs = rp.get_limb_names()
        if not valid_limbs:
            rp.log_message(("Cannot detect any limb parameters on this robot. "
                            "Exiting."), "ERROR")
            return
        
        rospy.loginfo("Valid Sawyer Limbs==>"+str(valid_limbs))
        
        print("Getting robot state... ")
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        
        rospy.loginfo("Enabling robot...")
        rs.enable()
        self.map_actions_to_movement()
        
        
    def map_actions_to_movement(self, side="right", joint_delta=0.1):
        
        self.limb = intera_interface.Limb(side)

        try:
            self.gripper = intera_interface.Gripper(side + '_gripper')
        except:
            self.has_gripper = False
            rospy.loginfo("The electric gripper is not detected on the robot.")
        else:
            self.has_gripper = True
    
        self.joints = limb.joint_names()
    
        self.bindings = {
            '1': (self.set_j, [self.joints[0], joint_delta], self.joints[0]+" increase"),
            'q': (self.set_j, [self.joints[0], -joint_delta], self.joints[0]+" decrease"),
            '2': (self.set_j, [self.joints[1], joint_delta], self.joints[1]+" increase"),
            'w': (self.set_j, [self.joints[1], -joint_delta], self.joints[1]+" decrease"),
            '3': (self.set_j, [self.joints[2], joint_delta], self.joints[2]+" increase"),
            'e': (self.set_j, [self.joints[2], -joint_delta], self.joints[2]+" decrease"),
            '4': (self.set_j, [self.joints[3], joint_delta], self.joints[3]+" increase"),
            'r': (self.set_j, [self.joints[3], -joint_delta], self.joints[3]+" decrease"),
            '5': (self.set_j, [self.joints[4], joint_delta], self.joints[4]+" increase"),
            't': (self.set_j, [self.joints[4], -joint_delta], self.joints[4]+" decrease"),
            '6': (self.set_j, [self.joints[5], joint_delta], self.joints[5]+" increase"),
            'y': (self.set_j, [self.joints[5], -joint_delta], self.joints[5]+" decrease"),
            '7': (self.set_j, [self.joints[6], joint_delta], self.joints[6]+" increase"),
            'u': (self.set_j, [self.joints[6], -joint_delta], self.joints[6]+" decrease")
         }
        if self.has_gripper:
            self.bindings.update({
            '8': (self.set_g, "close", side+" gripper close"),
            'i': (self.set_g, "open", side+" gripper open"),
            '9': (self.set_g, "calibrate", side+" gripper calibrate")
            })
        
        rospy.loginfo("Controlling joints...")
        
        
        
    def execute_movement(self, action_id):
        """
        It executed the command given through an id. This will move any joint 
        of Sawyer, including the gripper if it has it.
        :param: action_id: These are the possible action_id values and the action asociated.
        
        '1': self.joints[0]+" increase,
        'q': self.joints[0]+" decrease,
        '2': self.joints[1]+" increase,
        'w': self.joints[1]+" decrease,
        '3': self.joints[2]+" increase,
        'e': self.joints[2]+" decrease,
        '4': self.joints[3]+" increase,
        'r': self.joints[3]+" decrease,
        '5': self.joints[4]+" increase,
        't': self.joints[4]+" decrease,
        '6': self.joints[5]+" increase,
        'y': self.joints[5]+" decrease,
        '7': self.joints[6]+" increase,
        'u': self.joints[6]+" decrease,
        '8': gripper close"),
        'i': gripper open"),
        '9': gripper calibrate")
        """

        if c in self.bindings:
            cmd = self.bindings[c]
            if c == '8' or c == 'i' or c == '9':
                cmd[0](cmd[1])
                rospy.loginfo("command: %s" % (cmd[2],))
            else:
                #expand binding to something like "self.set_j(right, 'j0', joint_delta)"
                cmd[0](*cmd[1])
                rospy.loginfo("command: %s" % (cmd[2],))
        else:
            rospy.logerr("NOT VALID key binding, it should be one of these: ")
            for key, val in sorted(self.bindings.items(),
                                   key=lambda x: x[1][2]):
                rospy.logerr("  %s: %s" % (key, val[2]))

                        
    def set_j(self,joint_name, delta):
        current_position = self.limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        self.limb.set_joint_positions(joint_command)

    def set_g(self,action):
        if self.has_gripper:
            if action == "close":
                self.gripper.close()
            elif action == "open":
                self.gripper.open()
            elif action == "calibrate":
                self.gripper.calibrate()
    
    def get_all_limb_joint_angles(self):
        """
        Retunr array with all the joints angles
        """
        joints_angles_array = []
        for join_name in self.joints:
            joints_angles_array.append(self.get_limp_joint_angle(joint_name))
        return joints_angles_array
    
    def get_limp_joint_angle(self, joint_name):
        """
        Returns the angle of each joint searchable by name
        """
        return self.limb.joint_angle(joint_name)
        
        
        

    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()
    
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
        
    # Methods that the TrainingEnvironment will need.
    # ----------------------------
    