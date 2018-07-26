import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import sawyer_env
from gym.envs.registration import register
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

timestep_limit_per_episode = 10000 # Can be any Value

register(
        id='SawyerTouchCube-v0',
        entry_point='openai_ros:SawyerTouchCubeEnv',
        timestep_limit=timestep_limit_per_episode,
    )

class SawyerTouchCubeEnv(sawyer_env.sawyerEnv):
    def __init__(self):
        """
        Make sawyer learn how pick up a cube
        """
        
        # Only variable needed to be set here

        rospy.logdebug("Start SawyerTouchCubeEnv INIT...")
        number_actions = rospy.get_param('/sawyer/n_actions')
        self.action_space = spaces.Discrete(number_actions)
        
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)
        
        self.work_space_x_max = rospy.get_param("/sawyer/work_space/x_max")
        self.work_space_x_min = rospy.get_param("/sawyer/work_space/x_min")
        self.work_space_y_max = rospy.get_param("/sawyer/work_space/y_max")
        self.work_space_y_min = rospy.get_param("/sawyer/work_space/y_min")
        self.work_space_z_max = rospy.get_param("/sawyer/work_space/z_max")
        self.work_space_z_min = rospy.get_param("/sawyer/work_space/z_min")
        
        self.dec_obs = rospy.get_param("/sawyer/number_decimals_precision_obs")
        
        
        # We place the Maximum and minimum values of observations
        # TODO: Fill when get_observations is done.
        high = numpy.array([self.work_space_x_max,
                            self.work_space_y_max,
                            1.57,
                            1.57,
                            3.14,
                            self.propeller_high_speed,
                            self.propeller_high_speed,
                            self.max_angular_speed,
                            self.max_distance_from_des_point
                            ])
                                        
        low = numpy.array([ self.work_space_x_min,
                            self.work_space_y_min,
                            -1*1.57,
                            -1*1.57,
                            -1*3.14,
                            -1*self.propeller_high_speed,
                            -1*self.propeller_high_speed,
                            -1*self.max_angular_speed,
                            0.0
                            ])

        
        self.observation_space = spaces.Box(low, high)
        
        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # Rewards
        
        self.done_reward =rospy.get_param("/sawyer/done_reward")
        self.closer_to_point_reward = rospy.get_param("/sawyer/closer_to_point_reward")

        self.cumulated_steps = 0.0

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(SawyerTouchCubeEnv, self).__init__()
        
        rospy.logdebug("END SawyerTouchCubeEnv INIT...")

    def _set_init_pose(self):
        """
        Sets the two proppelers speed to 0.0 and waits for the time_sleep
        to allow the action to be executed
        """

        # We set the angles to zero of the limb
        self.joints = self.get_limb_joint_names_array()
        new_joint_angle = 0.0
        for joint_name in self.joints:
            self.set_joints_to_angle_directly(joint_name, new_joint_angle)
            
        # We Open the gripper
        self.set_g(action="open")

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
        translation_tcp_block, rotation_tcp_block = self.get_tf_start_to_end_frames(start_frame_name="right_electric_gripper_base",
                                                                                    end_frame_name="block")
        tf_tcp_to_block_vector = Vector3()
        tf_tcp_to_block_vector.x = translation_tcp_block[0]
        tf_tcp_to_block_vector.y = translation_tcp_block[1]
        tf_tcp_to_block_vector.z = translation_tcp_block[2]
        
        self.previous_distance_from_block = self.get_magnitud_tf_tcp_to_block(tf_tcp_to_block_vector)

        

    def _set_action(self, action):
        """
        It sets the joints of sawyer based on the action integer given
        based on the action number given.
        :param action: The action integer that sets what movement to do next.
        """
        
        rospy.logdebug("Start Set Action ==>"+str(action))
       
        
        if action == 0: # Increase joint_0
            action_id = self.joints[0]+"_increase"
        elif action == 1: # Decrease joint_0
            action_id = self.joints[0]+"_decrease"
        elif action == 2: # Increase joint_1
            action_id = self.joints[1]+"_increase"
        elif action == 3: # Decrease joint_1
            action_id = self.joints[1]+"_decrease"
        elif action == 4: # Increase joint_2
            action_id = self.joints[2]+"_increase"
        elif action == 5: # Decrease joint_2
            action_id = self.joints[2]+"_decrease"
        elif action == 6: # Increase joint_3
            action_id = self.joints[3]+"_increase"
        elif action == 7: # Decrease joint_3
           action_id = self.joints[3]+"_decrease"
        elif action == 8: # Increase joint_4
            action_id = self.joints[4]+"_increase"
        elif action == 9: # Decrease joint_4
            action_id = self.joints[4]+"_decrease"
        elif action == 10: # Increase joint_5
            action_id = self.joints[5]+"_increase"
        elif action == 11: # Decrease joint_5
            action_id = self.joints[5]+"_decrease"
        elif action == 12: # Increase joint_6
            action_id = self.joints[6]+"_increase"
        elif action == 13: # Decrease joint_6
            action_id = self.joints[6]+"_decrease"

        
        # We tell sawyer the propeller speeds
        self.set_propellers_speed(  right_propeller_speed,
                                    left_propeller_speed,
                                    time_sleep=1.0)
        
        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have access to, we need to read the
        sawyerEnv API DOCS.
        :return: observation
        """
        rospy.logdebug("Start Get Observation ==>")

        odom = self.get_odom()
        base_position = odom.pose.pose.position
        base_orientation_quat = odom.pose.pose.orientation
        base_roll, base_pitch, base_yaw = self.get_orientation_euler(base_orientation_quat)
        base_speed_linear = odom.twist.twist.linear
        base_speed_angular_yaw = odom.twist.twist.angular.z
        
        distance_from_desired_point = self.get_distance_from_desired_point(base_position)

        observation = []
        observation.append(round(base_position.x,self.dec_obs))
        observation.append(round(base_position.y,self.dec_obs))
        
        observation.append(round(base_roll,self.dec_obs))
        observation.append(round(base_pitch,self.dec_obs))
        observation.append(round(base_yaw,self.dec_obs))
        
        observation.append(round(base_speed_linear.x,self.dec_obs))
        observation.append(round(base_speed_linear.y,self.dec_obs))
        
        observation.append(round(base_speed_angular_yaw,self.dec_obs))
        
        observation.append(round(distance_from_desired_point,self.dec_obs))

        return observation
        

    def _is_done(self, observations):
        """
        We consider the episode done if:
        1) The sawyers is ouside the workspace
        2) It got to the desired point
        """
        distance_from_desired_point = observations[8]

        current_position = Vector3()
        current_position.x = observations[0]
        current_position.y = observations[1]
        
        is_inside_corridor = self.is_inside_workspace(current_position)
        has_reached_des_point = self.is_in_desired_position(current_position, self.desired_point_epsilon)
        
        done = not(is_inside_corridor) or has_reached_des_point
        
        return done

    def _compute_reward(self, observations, done):
        """
        We Base the rewards in if its done or not and we base it on
        if the distance to the desired point has increased or not
        :return:
        """

        # We only consider the plane, the fluctuation in z is due mainly to wave
        current_position = Point()
        current_position.x = observations[0]
        current_position.y = observations[1]
        
        distance_from_des_point = self.get_distance_from_desired_point(current_position)
        distance_difference =  distance_from_des_point - self.previous_distance_from_des_point


        if not done:
            
            # If there has been a decrease in the distance to the desired point, we reward it
            if distance_difference < 0.0:
                rospy.logwarn("DECREASE IN DISTANCE GOOD")
                reward = self.closer_to_point_reward
            else:
                rospy.logerr("ENCREASE IN DISTANCE BAD")
                reward = -1*self.closer_to_point_reward

        else:
            
            if self.is_in_desired_position(current_position, self.desired_point_epsilon):
                reward = self.done_reward
            else:
                reward = -1*self.done_reward


        self.previous_distance_from_des_point = distance_from_des_point


        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

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
        
        rospy.logdebug("###### IS DESIRED POS ? ######")
        rospy.logdebug("current_position"+str(current_position))
        rospy.logdebug("x_pos_plus"+str(x_pos_plus)+",x_pos_minus="+str(x_pos_minus))
        rospy.logdebug("y_pos_plus"+str(y_pos_plus)+",y_pos_minus="+str(y_pos_minus))
        rospy.logdebug("x_pos_are_close"+str(x_pos_are_close))
        rospy.logdebug("y_pos_are_close"+str(y_pos_are_close))
        rospy.logdebug("is_in_desired_pos"+str(is_in_desired_pos))
        rospy.logdebug("############")
        
        return is_in_desired_pos
    
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
    
    def get_magnitud_tf_tcp_to_block(self, translation_vector):
        """
        Given a Vector3 Object, get the magnitud
        :param p_end:
        :return:
        """
        a = numpy.array((   translation_vector.x,
                            translation_vector.y,
                            translation_vector.z))
        
        distance = numpy.linalg.norm(a)
    
        return distance
        
    def get_orientation_euler(self, quaternion_vector):
        # We convert from quaternions to euler
        orientation_list = [quaternion_vector.x,
                            quaternion_vector.y,
                            quaternion_vector.z,
                            quaternion_vector.w]
    
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw
        
    def is_inside_workspace(self,current_position):
        """
        Check if the sawyer is inside the Workspace defined
        """
        is_inside = False

        rospy.logwarn("##### INSIDE WORK SPACE? #######")
        rospy.logwarn("XYZ current_position"+str(current_position))
        rospy.logwarn("work_space_x_max"+str(self.work_space_x_max)+",work_space_x_min="+str(self.work_space_x_min))
        rospy.logwarn("work_space_y_max"+str(self.work_space_y_max)+",work_space_y_min="+str(self.work_space_y_min))
        rospy.logwarn("############")

        if current_position.x > self.work_space_x_min and current_position.x <= self.work_space_x_max:
            if current_position.y > self.work_space_y_min and current_position.y <= self.work_space_y_max:
                    is_inside = True
        
        return is_inside
        
    
