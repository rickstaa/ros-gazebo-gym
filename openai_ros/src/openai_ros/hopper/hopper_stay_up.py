import rospy
import numpy
from gym import spaces
from openai_ros import hopper_env
from gym.envs.registration import register
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

timestep_limit_per_episode = 10000 # Can be any Value

register(
        id='HopperStayUp-v0',
        entry_point='openai_ros:HopperStayUpEnv',
        timestep_limit=timestep_limit_per_episode,
    )

class HopperStayUpEnv(hopper_env.HopperEnv):
    def __init__(self):
        """
        Make Hopper Learn how to Stay Up indefenitly
        """
        
        # Only variable needed to be set here
        """
        For this version, we consider 6 actions
        1-2) Increment/Decrement haa_joint
        3-4) Increment/Decrement hfe_joint
        5-6) Increment/Decrement kfe_joint
        """
        number_actions = rospy.get_param('/monoped/n_actions')
        self.action_space = spaces.Discrete(number_actions)
        
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)
        
        
        # Actions and Observations
        
        self.init_joint_states = Vector3()
        self.init_joint_states.x = rospy.get_param('/monoped/init_joint_states/haa_joint')
        self.init_joint_states.y = rospy.get_param('/monoped/init_joint_states/hfe_joint')
        self.init_joint_states.z = rospy.get_param('/monoped/init_joint_states/kfe_joint')
        
        
        # Get Desired Point to Get
        self.desired_point = Point()
        self.desired_point.x = rospy.get_param("/monoped/desired_point/x")
        self.desired_point.y = rospy.get_param("/monoped/desired_point/y")
        self.desired_point.z = rospy.get_param("/monoped/desired_point/z")
        
        
        self.joint_increment_value = rospy.get_param("/monoped/joint_increment_value")
        
        
        self.dec_obs = rospy.get_param("/monoped/number_decimals_precision_obs")
        
        self.min_height = rospy.get_param("/monoped/min_height")
        self.max_height = rospy.get_param("/monoped/max_height")
        
        self.distance_from_desired_point_max = rospy.get_param("/monoped/distance_from_desired_point_max")
        
        self.max_incl = rospy.get_param("/monoped/max_incl")
        self.max_contact_force = rospy.get_param("/monoped/max_contact_force")
        
        self.maximum_haa_joint = rospy.get_param("/monoped/max_incl")
        self.maximum_hfe_joint = rospy.get_param("/monoped/max_incl")
        self.maximum_kfe_joint = rospy.get_param("/monoped/max_incl")
        self.min_kfe_joint = rospy.get_param("/monoped/max_incl")
        
        # We place the Maximum and minimum values of observations

        
        high = numpy.array([self.distance_from_desired_point_max,
                            self.max_incl,
                            self.max_incl,
                            3.14,
                            self.max_contact_force,
                            self.maximum_haa_joint,
                            self.maximum_hfe_joint,
                            self.maximum_kfe_joint,
                            self.max_height])
                                        
        low = numpy.array([ 0.0,
                            -1*self.max_incl,
                            -1*self.max_incl,
                            -1*3.14,
                            0.0,
                            self.maximum_haa_joint,
                            self.maximum_hfe_joint,
                            self.min_kfe_joint,
                            self.min_height])

        
        self.observation_space = spaces.Box(low, high)
        
        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # Rewards
        # TODO

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(HopperStayUpEnv, self).__init__()

    def _set_init_pose(self):
        """
        Sets the Robot in its init linear and angular speeds
        and lands the robot. Its preparing it to be reseted in the world.
        """

        joints_array = [self.init_joint_states.x,
                        self.init_joint_states.y,
                        self.init_joint_states.z]
        
        self.move_joints(   joints_array,
                            epsilon=0.05,
                            update_rate=10)

        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """

        # For Info Purposes
        self.cumulated_reward = 0.0
        # We get the initial pose to mesure the distance from the desired point.
        odom = self.get_odom()
        self.previous_distance_from_des_point = self.get_distance_from_desired_point(odom.pose.pose.position)

        

    def _set_action(self, action):
        """
        It sets the joints of monoped based on the action integer given
        based on the action number given.
        :param action: The action integer that sets what movement to do next.
        """
        
        rospy.logdebug("Start Set Action ==>"+str(action))
       
        # We get current Joints values
        joint_states = self.get_joint_states()
        joint_states_position = joint_states.position
        rospy.logdebug("get_action_to_position>>>"+str(joint_states_position))
        
        action_position = [0.0,0.0,0.0]
        if action == 0: #Increment haa_joint
            action_position[0] = joint_states_position[0] + self.joint_increment_value
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
        elif action == 1: #Decrement haa_joint
            action_position[0] = joint_states_position[0] - self.joint_increment_value
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
        elif action == 2: #Increment hfe_joint
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1] + self.joint_increment_value
            action_position[2] = joint_states_position[2]
        elif action == 3: #Decrement hfe_joint
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1] - self.joint_increment_value
            action_position[2] = joint_states_position[2]
        elif action == 4: #Increment kfe_joint
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2] + self.joint_increment_value
        elif action == 5:  #Decrement kfe_joint
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2] - self.joint_increment_value

        
        # We tell monoped where to place its joints next
        self.move_joints(   joints_array,
                            epsilon=0.05,
                            update_rate=10)
        
        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have access to, we need to read the
        HopperEnv API DOCS
        Returns the state of the robot needed for OpenAI QLearn Algorithm
        The state will be defined by an array of the:
        1) distance from desired point in meters
        2) The pitch orientation in radians
        3) the Roll orientation in radians
        4) the Yaw orientation in radians
        5) Force in contact sensor in Newtons
        6-7-8) State of the 3 joints in radians
        9) Height of the Base

        observation = [distance_from_desired_point,
                 base_roll,
                 base_pitch,
                 base_yaw,
                 contact_force,
                 joint_states_haa,
                 joint_states_hfe,
                 joint_states_kfe,
                 height_base]
        :return: observation
        """
        rospy.logdebug("Start Get Observation ==>")
        
        distance_from_desired_point = self.get_distance_from_desired_point(self.desired_world_point)

        base_orientation = self.get_base_rpy()
        base_roll = base_orientation.x
        base_pitch = base_orientation.y
        base_yaw = base_orientation.z

        contact_force = self.get_contact_force_magnitude()

        joint_states = self.get_joint_states()
        joint_states_haa = joint_states.position[0]
        joint_states_hfe = joint_states.position[1]
        joint_states_kfe = joint_states.position[2]
        
        odom = self.get_odom()
        base_position = odom.pose.pose.position
        height_base = get_base_height(base_position)

        observation = []
        observation.append(round(distance_from_desired_point,self.dec_obs))
        observation.append(round(base_roll,self.dec_obs))
        observation.append(round(base_pitch,self.dec_obs))
        observation.append(round(base_yaw,self.dec_obs))
        observation.append(round(contact_force,self.dec_obs))
        observation.append(round(joint_states_haa,self.dec_obs)
        observation.append(round(joint_states_hfe,self.dec_obs))
        observation.append(round(joint_states_kfe,self.dec_obs))
        observation.append(round(joint_states_kfe,self.dec_obs))
        observation.append(round(height_base,self.dec_obs))

        return observation
        

    def _is_done(self, observations):
        """
        We consider the episode done if:
        1) The Monopeds height is lower than a threshhold
        2) The Orientation is outside a threshold
        """
        
        monoped_height_ok = self.monoped_height_ok()
        monoped_orientation_ok = self.monoped_orientation_ok()

        done = not(monoped_height_ok and monoped_orientation_ok)
        
        return done

    def _compute_reward(self, observations, done):

        current_position = Point()
        current_position.x = observations[0]
        current_position.y = observations[1]
        current_position.z = observations[2]

        distance_from_des_point = self.get_distance_from_desired_point(current_position)
        distance_difference =  distance_from_des_point - self.previous_distance_from_des_point


        if not done:
            
            # If there has been a decrease in the distance to the desired point, we reward it
            if distance_difference < 0.0:
                rospy.logwarn("DECREASE IN DISTANCE GOOD")
                reward = self.closer_to_point_reward
            else:
                rospy.logerr("ENCREASE IN DISTANCE BAD")
                reward = 0
                
        else:
            
            if self.is_in_desired_position(current_position, epsilon=0.5):
                reward = self.end_episode_points
            else:
                reward = -1*self.end_episode_points


        self.previous_distance_from_des_point = distance_from_des_point


        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))

        return reward


    # Internal TaskEnv Methods
    
    def is_in_desired_position(self,current_position, epsilon=0.05):
        """
        It return True if the current position is similar to the desired poistion
        """
        
        is_in_desired_pos = False
        
        
        x_pos_plus = self.desired_point.x + epsilon
        x_pos_minus = self.desired_point.x - epsilon
        y_pos_plus = self.desired_point.y + epsilon
        y_pos_minus = self.desired_point.y - epsilon
        
        x_current = current_position.x
        y_current = current_position.y
        
        x_pos_are_close = (x_current <= x_pos_plus) and (x_current > x_pos_minus)
        y_pos_are_close = (y_current <= y_pos_plus) and (y_current > y_pos_minus)
        
        is_in_desired_pos = x_pos_are_close and y_pos_are_close
        
        rospy.logwarn("###### IS DESIRED POS ? ######")
        rospy.logwarn("current_position"+str(current_position))
        rospy.logwarn("x_pos_plus"+str(x_pos_plus)+",x_pos_minus="+str(x_pos_minus))
        rospy.logwarn("y_pos_plus"+str(y_pos_plus)+",y_pos_minus="+str(y_pos_minus))
        rospy.logwarn("x_pos_are_close"+str(x_pos_are_close))
        rospy.logwarn("y_pos_are_close"+str(y_pos_are_close))
        rospy.logwarn("is_in_desired_pos"+str(is_in_desired_pos))
        rospy.logwarn("############")
        
        return is_in_desired_pos
    
    def is_inside_workspace(self,current_position):
        """
        Check if the monoped is inside the Workspace defined
        """
        is_inside = False

        rospy.logwarn("##### INSIDE WORK SPACE? #######")
        rospy.logwarn("XYZ current_position"+str(current_position))
        rospy.logwarn("work_space_x_max"+str(self.work_space_x_max)+",work_space_x_min="+str(self.work_space_x_min))
        rospy.logwarn("work_space_y_max"+str(self.work_space_y_max)+",work_space_y_min="+str(self.work_space_y_min))
        rospy.logwarn("work_space_z_max"+str(self.work_space_z_max)+",work_space_z_min="+str(self.work_space_z_min))
        rospy.logwarn("############")

        if current_position.x > self.work_space_x_min and current_position.x <= self.work_space_x_max:
            if current_position.y > self.work_space_y_min and current_position.y <= self.work_space_y_max:
                if current_position.z > self.work_space_z_min and current_position.z <= self.work_space_z_max:
                    is_inside = True
        
        return is_inside
        
    def sonar_detected_something_too_close(self, sonar_value):
        """
        Detects if there is something too close to the monoped front
        """
        rospy.logwarn("##### SONAR TOO CLOSE? #######")
        rospy.logwarn("sonar_value"+str(sonar_value)+",min_sonar_value="+str(self.min_sonar_value))
        rospy.logwarn("############")
        
        too_close = sonar_value < self.min_sonar_value
        
        return too_close
        
    def monoped_has_flipped(self,current_orientation):
        """
        Based on the orientation RPY given states if the monoped has flipped
        """
        has_flipped = True
        
        
        self.max_roll = rospy.get_param("/monoped/max_roll")
        self.max_pitch = rospy.get_param("/monoped/max_pitch")
        
        rospy.logwarn("#### HAS FLIPPED? ########")
        rospy.logwarn("RPY current_orientation"+str(current_orientation))
        rospy.logwarn("max_roll"+str(self.max_roll)+",min_roll="+str(-1*self.max_roll))
        rospy.logwarn("max_pitch"+str(self.max_pitch)+",min_pitch="+str(-1*self.max_pitch))
        rospy.logwarn("############")
        
        
        if current_orientation.x > -1*self.max_roll and current_orientation.x <= self.max_roll:
            if current_orientation.y > -1*self.max_pitch and current_orientation.y <= self.max_pitch:
                    has_flipped = False
        
        return has_flipped
        
    def get_distance_from_desired_point(self, current_position):
        """
        Calculates the distance from the current position to the desired point
        :param start_point:
        :return:
        """
        distance = self.get_distance_from_point(current_position,
                                                self.desired_point)
    
        return distance
    
    def get_distance_from_point(self, pstart, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((pstart.x, pstart.y, pstart.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))
    
        distance = numpy.linalg.norm(a - b)
    
        return distance
        
    def get_orientation_euler(self, quaternion_vector):
        # We convert from quaternions to euler
        orientation_list = [quaternion_vector.x,
                            quaternion_vector.y,
                            quaternion_vector.z,
                            quaternion_vector.w]
    
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw
        
    def monoped_height_ok(self):

        height_ok = self.min_height <= self.get_base_height() < self.max_height
        return height_ok
        
    def get_base_height(self, base_position):
        return base_position.z
        
    def get_base_rpy(self):
        
        imu = self.get_imu()
        base_orientation = imu.orientation
        
        euler_rpy = Vector3()
        euler = euler_from_quaternion([ base_orientation.x,
                                        base_orientation.y,
                                        base_orientation.z,
                                        base_orientation.w]
                                        )
        euler_rpy.x = euler[0]
        euler_rpy.y = euler[1]
        euler_rpy.z = euler[2]
        
        return euler_rpy
        
    def get_contact_force_magnitude(self):
        """
        You will see that because the X axis is the one pointing downwards, it will be the one with
        higher value when touching the floor
        For a Robot of total mas of 0.55Kg, a gravity of 9.81 m/sec**2, Weight = 0.55*9.81=5.39 N
        Falling from around 5centimetres ( negligible height ), we register peaks around
        Fx = 7.08 N
        :return:
        """
        # We get the Contact Sensor data
        lowerleg_contactsensor_state = self.get_lowerleg_contactsensor_state()
        # We extract what we need that is only the total_wrench force
        contact_force = self.get_contact_force(lowerleg_contactsensor_state)
        # We create an array with each component XYZ
        contact_force_np = numpy.array((contact_force.x, contact_force.y, contact_force.z))
        # We calculate the magnitude of the Force Vector, array.
        force_magnitude = numpy.linalg.norm(contact_force_np)

        return force_magnitude
        
    def get_contact_force(self, lowerleg_contactsensor_state):
        """
        /lowerleg_contactsensor_state/states[0]/contact_positions ==> PointContact in World
        /lowerleg_contactsensor_state/states[0]/contact_normals ==> NormalContact in World

        ==> One is an array of all the forces, the other total,
         and are relative to the contact link referred to in the sensor.
        /lowerleg_contactsensor_state/states[0]/wrenches[]
        /lowerleg_contactsensor_state/states[0]/total_wrench
        :return:
        """
        contact_force = None
        for state in lowerleg_contactsensor_state.states:
            self.contact_force = state.total_wrench.force
        
        return contact_force

