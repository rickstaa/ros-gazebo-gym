import numpy
import rospy
from openai_gazebo import robot_gazebo_env
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class CubeSingleDiskEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all CubeSingleDisk environments.
    """

    def __init__(self, n_actions, init_roll_vel):
        """Initializes a new CubeSingleDisk environment.

        Args:
            test_cubesinglediskenc_arg: Just to test that we can pass args to this env init.
            n_actions: Number of actions to perform
        """
        # Variables that we give through the constructor.
        self.init_roll_vel = init_roll_vel

        self.controllers_list = ['joint_state_controller',
                                 'inertia_wheel_roll_joint_velocity_controller'
                                 ]

        self.robot_name_space = "moving_cube"

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(CubeSingleDiskEnv, self).__init__(n_actions=n_actions,
                                                controllers_list=self.controllers_list,
                                                robot_name_space=self.robot_name_space,
                                                reset_controls=True)



        """
        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.
        """
        self.gazebo.unpauseSim()
        self.controllers_object.reset_controllers()
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber("/moving_cube/joint_states", JointState, self._joints_callback)
        rospy.Subscriber("/moving_cube/odom", Odometry, self._odom_callback)

        self._roll_vel_pub = rospy.Publisher('/moving_cube/inertia_wheel_roll_joint_velocity_controller/command',
                                             Float64, queue_size=1)

        self._check_publishers_connection()

        self.gazebo.pauseSim()



    # GoalEnv methods
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_joints(self.init_roll_vel)

        return True

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True


    def _check_all_sensors_ready(self):
        self._check_joint_states_ready()
        self._check_odom_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message("/moving_cube/joint_states", JointState, timeout=1.0)
                rospy.logdebug("Current moving_cube/joint_states READY=>" + str(self.joints))

            except:
                rospy.logerr("Current moving_cube/joint_states not ready yet, retrying for getting joint_states")
        return self.joints

    def _check_odom_ready(self):
        self.odom = None
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/moving_cube/odom", Odometry, timeout=1.0)
                rospy.logdebug("Current /moving_cube/odom READY=>" + str(self.odom))

            except:
                rospy.logerr("Current /moving_cube/odom not ready yet, retrying for getting odom")

        return self.odom

    def _joints_callback(self, data):
        self.joints = data

    def _odom_callback(self, data):
        self.odom = data

    def get_joints(self):
        return self.joints

    def get_odom(self):
        return self.odom

    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self._roll_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to _roll_vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_roll_vel_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def move_joints(self, roll_speed):

        joint_speed_value = Float64()
        joint_speed_value.data = roll_speed
        rospy.logdebug("Single Disk Roll Velocity>>" + str(joint_speed_value))
        self._roll_vel_pub.publish(joint_speed_value)
        self.wait_until_roll_is_in_vel(joint_speed_value.data)

    def wait_until_roll_is_in_vel(self, velocity):

        rate = rospy.Rate(10)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0
        epsilon = 0.1
        v_plus = velocity + epsilon
        v_minus = velocity - epsilon
        while not rospy.is_shutdown():
            joint_data = self._check_joint_states_ready()
            roll_vel = joint_data.velocity[0]
            rospy.logdebug("VEL=" + str(roll_vel) + ", ?RANGE=[" + str(v_minus) + ","+str(v_plus)+"]")
            are_close = (roll_vel <= v_plus) and (roll_vel > v_minus)
            if are_close:
                rospy.logdebug("Reached Velocity!")
                end_wait_time = rospy.get_rostime().to_sec()
                break
            rospy.logdebug("Not there yet, keep waiting...")
            rate.sleep()
        delta_time = end_wait_time- start_wait_time
        rospy.logdebug("[Wait Time=" + str(delta_time)+"]")
        return delta_time


    def get_distance_from_start_point(self, start_point):
        """
        Calculates the distance from the given point and the current position
        given by odometry
        :param start_point:
        :return:
        """
        distance = self.get_distance_from_point(start_point,
                                                self.odom.pose.pose.position)

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

    def get_orientation_euler(self):
        # We convert from quaternions to euler
        orientation_list = [self.odom.pose.pose.orientation.x,
                            self.odom.pose.pose.orientation.y,
                            self.odom.pose.pose.orientation.z,
                            self.odom.pose.pose.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw

    def get_roll_velocity(self):
        # We get the current joint roll velocity
        roll_vel = self.joints.velocity[0]
        return roll_vel

    # RobotEnv methods
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

    def _convert_obs_to_state(self, observations):
        """Converts the observations used for reward and so on to the essentials for the robot state
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