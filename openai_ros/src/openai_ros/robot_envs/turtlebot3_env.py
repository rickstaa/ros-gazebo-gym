#!/usr/bin/env python3
"""Robot environment for the Turtlebot3. This is the new version of the classic
Turtlebot2 created by `ROBOTIS <https://www.robotis.us>`__.
"""
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from openai_ros.core import ROSLauncher
from openai_ros.robot_gazebo_env import RobotGazeboEnv
from sensor_msgs.msg import Imu, LaserScan


class TurtleBot3Env(RobotGazeboEnv):
    """Turtlebot3 robot environment class.

    Attributes:
        controllers_list (list): The controllers that are used.
        robot_name_space (str): The robot namespace that is used.
        gazebo (:class:`openai_ros.core.gazebo_connection.GazeboConnection`): Object
            that can be used to talk with the gazebo simulator.
    """

    def __init__(self, workspace_path=None):
        """Initializes a new TurtleBot3Env robot environment.

        TurtleBot3 doesn't use controller_manager, therefore we won't reset the
        controllers in the standard fashion. For the moment we won't reset them.

        To check any topic we need to have the simulations running, we need to do two
        things:
            1. Un-pause the simulation: without that th stream of data doesn't flow.
               This is for simulations that are pause for whatever the reason
            2. If the simulation was running already for some reason, we need to reset
               the controllers.

        This has to do with the fact that some plugins with tf, don't understand the
        reset of the simulation and need to be reset to work properly.

        The Sensors: The sensors accessible are the ones considered usefull for AI
        learning.

        Sensor Topic List:
            * /odom: Odometry readings of the base of the Robot.
            * /imu: Inertial Measuring Unit that gives relative accelerations and
              orientations.
            * /scan: Laser readings.

        Actuators Topic List: /cmd_vel

        Args:
            workspace_path (str, optional): The path of the workspace in which the
                turtlebot3_gazebo package should be found. Defaults to ``None``.
        """
        rospy.logdebug("Initialize TurtleBot3Env robot environment...")

        # Launch the ROS launch that spawns the robot into the world
        ROSLauncher.launch(
            package_name="turtlebot3_gazebo",
            launch_file_name="put_robot_in_world.launch",
            workspace_path=workspace_path,
        )

        # Setup internal robot environment variables
        self.controllers_list = ["imu"]
        self.robot_name_space = ""

        # Initialize gazebo environment
        super(TurtleBot3Env, self).__init__(
            controllers_list=self.controllers_list,
            robot_name_space=self.robot_name_space,
            reset_controls=False,
        )

        # Un-pause simulations, reset controllers and check robot sensors
        self.gazebo.unpause_sim()
        # self.controllers_object.reset_controllers()

        # Create ROS related Subscribers and publishers
        self._check_all_sensors_ready()
        rospy.Subscriber("/odom", Odometry, self._odom_callback)
        rospy.Subscriber("/imu", Imu, self._imu_callback)
        rospy.Subscriber("/scan", LaserScan, self._laser_scan_callback)
        self._cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._check_publishers_connection()

        # Pause the simulation
        self.gazebo.pause_sim()
        rospy.logdebug("TurtleBot3Env robot environment initialized.")

    ################################################
    # Robot env main methods #######################
    ################################################
    # NOTE: Contains methods that the TrainingEnvironment will need.
    def move_base(self, linear_speed, angular_speed, epsilon=0.05, update_rate=10):
        """Move the robot base iwth a given linear and angular speed.

        .. note::
            IThis method will move the base based on the linear and angular speeds
            given. It will wait until those twists are achieved reading from the
            odometry topic.

        Args:
            linear_speed (float): Speed in the X axis of the robot base frame.
            angular_speed (float):  Speed of the angular turning of the robot base
                frame.
            epsilon (float, optional): Acceptable difference between the speed asked
                and the odometry readings. Defaults to ``0.05``.
            update_rate (int, optional): Rate at which we check the odometry. Defaults
                to ``10``.
        """
        cmd_vel_value = Twist()
        cmd_vel_value.linear.x = linear_speed
        cmd_vel_value.angular.z = angular_speed
        rospy.logdebug("TurtleBot3 Base Twist Cmd>>" + str(cmd_vel_value))
        self._check_publishers_connection()
        self._cmd_vel_pub.publish(cmd_vel_value)

        # Wait till the desired twist is achieved
        # FIXME: The wait_until_twist_achieved function is currently not working
        # self.wait_until_twist_achieved(cmd_vel_value,epsilon,update_rate)
        time.sleep(0.2)  # Wait some time since above is broken

    def wait_until_twist_achieved(self, cmd_vel_value, epsilon, update_rate):
        """Wait for the cmd_vel twist given to be reached by the robot reading
        from the odometry.

        Args:
            cmd_vel_value (:obj:`geometry_msgs.msg.Twist`): Twist we want to wait to
                reach.
            epsilon (float): Error acceptable in odometry readings.
            update_rate (float): Rate at which we check the odometry.

        Returns:
            float: The time that passed.
        """
        rospy.logdebug("START wait_until_twist_achieved...")
        rate = rospy.Rate(update_rate)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0
        epsilon = 0.05
        rospy.logdebug("Desired Twist Cmd>>" + str(cmd_vel_value))
        rospy.logdebug("epsilon>>" + str(epsilon))

        # Unpack twist massage
        linear_speed = cmd_vel_value.linear.x
        angular_speed = cmd_vel_value.angular.z
        linear_speed_plus = linear_speed + epsilon
        linear_speed_minus = linear_speed - epsilon
        angular_speed_plus = angular_speed + epsilon
        angular_speed_minus = angular_speed - epsilon

        # Wait till desired twist is found
        while not rospy.is_shutdown():
            current_odometry = self._check_odom_ready()
            # IN turtlebot3 the odometry angular readings are inverted, so we have to
            # invert the sign.
            odom_linear_vel = current_odometry.twist.twist.linear.x
            odom_angular_vel = -1 * current_odometry.twist.twist.angular.z
            rospy.logdebug(
                "Linear VEL="
                + str(odom_linear_vel)
                + ", ?RANGE=["
                + str(linear_speed_minus)
                + ","
                + str(linear_speed_plus)
                + "]"
            )
            rospy.logdebug(
                "Angular VEL="
                + str(odom_angular_vel)
                + ", ?RANGE=["
                + str(angular_speed_minus)
                + ","
                + str(angular_speed_plus)
                + "]"
            )
            linear_vel_are_close = (odom_linear_vel <= linear_speed_plus) and (
                odom_linear_vel > linear_speed_minus
            )
            angular_vel_are_close = (odom_angular_vel <= angular_speed_plus) and (
                odom_angular_vel > angular_speed_minus
            )
            if linear_vel_are_close and angular_vel_are_close:
                rospy.logdebug("Reached Velocity!")
                end_wait_time = rospy.get_rostime().to_sec()
                break
            rospy.logdebug("Not there yet, keep waiting...")
            rate.sleep()
        delta_time = end_wait_time - start_wait_time
        rospy.logdebug("[Wait Time=" + str(delta_time) + "]")
        rospy.logdebug("END wait_until_twist_achieved...")
        return delta_time

    def get_odom(self):
        """Retrieve the robot odometry.

        Returns:
            :obj:`nav_msgs.msg._Odometry.Odometry`: The odometry message.
        """
        return self.odom

    def get_imu(self):
        """Retrieve the IMU data.

        Returns:
            :obj:`sensor_msgs.msg._Imu.Imu`: The imu data message.
        """
        return self.imu

    def get_laser_scan(self):
        """Retrieve the laser scan data.

        Returns:
            :obj:`sensor_msgs.msg._LaserScan.LaserScan`: The laser scan message.
        """
        return self.laser_scan

    ################################################
    # Panda Robot env helper methods ###############
    ################################################

    def _check_all_sensors_ready(self):
        """Check if all sensors are ready."""
        rospy.logdebug("START ALL SENSORS READY")
        self._check_odom_ready()
        self._check_imu_ready()
        self._check_laser_scan_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_odom_ready(self):
        """Checks if the odometry sensor is ready and wait till this it is.

        Returns:
            :obj:`nav_msgs.msg._Odometry.Odometry`: Odometry message.
        """
        self.odom = None
        rospy.logdebug("Waiting for /odom to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/odom", Odometry, timeout=5.0)
                rospy.logdebug("Current /odom READY=>")
            except Exception:
                rospy.logerr("Current /odom not ready yet, retrying for getting odom")
        return self.odom

    def _check_imu_ready(self):
        """Checks if the Inertial Measurement Unit is ready and wait till this it is.

        Returns:
            :obj:`sensor_msgs.msg._Imu.Imu`: IMU message.
        """
        self.imu = None
        rospy.logdebug("Waiting for /imu to be READY...")
        while self.imu is None and not rospy.is_shutdown():
            try:
                self.imu = rospy.wait_for_message("/imu", Imu, timeout=5.0)
                rospy.logdebug("Current /imu READY=>")
            except Exception:
                rospy.logerr("Current /imu not ready yet, retrying for getting imu")
        return self.imu

    def _check_laser_scan_ready(self):
        """Checks if the laser scanner is ready and wait till this it is.

        Returns:
            :obj:`sensor_msgs.msg._LaserScan.LaserScan`: Laser scanner message.
        """
        self.laser_scan = None
        rospy.logdebug("Waiting for /scan to be READY...")
        while self.laser_scan is None and not rospy.is_shutdown():
            try:
                self.laser_scan = rospy.wait_for_message(
                    "/scan", LaserScan, timeout=1.0
                )
                rospy.logdebug("Current /scan READY=>")
            except Exception:
                rospy.logerr(
                    "Current /scan not ready yet, retrying for getting laser_scan"
                )
        return self.laser_scan

    def _odom_callback(self, data):
        """Odometry subscriber callback function.

        Args:
            data (:obj:`nav_msgs.msg._Odometry.Odometry`): The data that is returned by
                the subscriber.
        """
        self.odom = data

    def _imu_callback(self, data):
        """IMU subscriber callback function.

        Args:
            data (:obj:`sensor_msgs.msg._Imu.Imu`): The data that is returned by
                the subscriber.
        """
        self.imu = data

    def _laser_scan_callback(self, data):
        """Laser scanner subscriber callback function.

        Args:
            data (:obj:`sensor_msgs.msg._LaserScan.LaserScan`): The data that is
                returned by the subscriber.
        """
        self.laser_scan = data

    def _check_publishers_connection(self):
        """Checks that all the publishers are working"""
        rate = rospy.Rate(10)  # 10hz
        while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to _cmd_vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_cmd_vel_pub Publisher Connected")
        rospy.logdebug("All Publishers READY")

    ################################################
    # Overload Gazebo env virtual methods ##########
    ################################################
    # NOTE: Methods needed by the gazebo environment
    def _check_all_systems_ready(self):
        """Checks that all the sensors, publishers and other simulation systems are
        operational.

        Returns:
            bool: Whether the systems are ready. Will not return if the systems are not
                yet ready.
        """
        self._check_all_sensors_ready()
        self._check_publishers_connection()
        rospy.logdebug("ALL SYSTEMS READY")
        return True
