import numpy
import rospy
#from gym.envs.robotics import rotations, robot_env, utils
from gym.envs.robotics import rotations, utils
from openai_gazebo import robot_gazebo_env
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion

def goal_distance(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return numpy.linalg.norm(goal_a - goal_b, axis=-1)


class CubeSingleDiskEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all CubeSingleDisk environments.
    """

    def __init__(
        self, test_cubesinglediskenc_arg, n_actions
    ):
        """Initializes a new CubeSingleDisk environment.

        Args:
            test_cubesinglediskenc_arg: Just to test that we can pass args to this env init.
            n_actions: Number of actions to perform
        """
        # Variables that we give through the constructor.
        self.test_cubesinglediskenc_arg = test_cubesinglediskenc_arg
        self.n_actions = n_actions

        # Variables that we retrieve through the param server, loded when launch training launch.
        self.wait_time = rospy.get_param('/moving_cube/wait_time')

        # We Start all the ROS related Subscribers and publishers
        self._check_all_sensors_ready()
        rospy.Subscriber("/moving_cube/joint_states", JointState, self._joints_callback)
        rospy.Subscriber("/moving_cube/odom", Odometry, self._odom_callback)

        self._roll_vel_pub = rospy.Publisher('/moving_cube/inertia_wheel_roll_joint_velocity_controller/command',
                                             Float64, queue_size=1)

        self._check_publishers_connection()

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(CubeSingleDiskEnv, self).__init__(n_actions=self.n_actions)

    # GoalEnv methods
    # ----------------------------
    def _check_all_sensors_ready(self):
        self.disk_joints_data = None
        while self.disk_joints_data is None and not rospy.is_shutdown():
            try:
                self.disk_joints_data = rospy.wait_for_message("/moving_cube/joint_states", JointState, timeout=1.0)
                rospy.loginfo("Current moving_cube/joint_states READY=>" + str(self.disk_joints_data))

            except:
                rospy.logerr("Current moving_cube/joint_states not ready yet, retrying for getting joint_states")

        self.cube_odom_data = None
        while self.disk_joints_data is None and not rospy.is_shutdown():
            try:
                self.cube_odom_data = rospy.wait_for_message("/moving_cube/odom", Odometry, timeout=1.0)
                rospy.loginfo("Current /moving_cube/odom READY=>" + str(self.cube_odom_data))

            except:
                rospy.logerr("Current /moving_cube/odom not ready yet, retrying for getting odom")
        rospy.loginfo("ALL SENSORS READY")

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
            rospy.loginfo("No susbribers to _roll_vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.loginfo("_base_pub Publisher Connected")

        rospy.loginfo("All Publishers READY")

    def move_joints(self, roll_speed):

        joint_speed_value = Float64()
        joint_speed_value.data = roll_speed
        rospy.loginfo("Single Disk Roll Velocity>>" + str(joint_speed_value))
        self._roll_vel_pub.publish(joint_speed_value)

    def get_cube_state(self):

        # We convert from quaternions to euler
        orientation_list = [self.odom.pose.pose.orientation.x,
                            self.odom.pose.pose.orientation.y,
                            self.odom.pose.pose.orientation.z,
                            self.odom.pose.pose.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        # We get the distance from the origin
        start_position = Point()
        start_position.x = 0.0
        start_position.y = 0.0
        start_position.z = 0.0

        distance = self.get_distance_from_point(start_position,
                                                self.odom.pose.pose.position)

        cube_state = [
            round(self.joints.velocity[0], 1),
            round(distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(yaw, 1)
        ]

        return cube_state

    def _compute_reward(self, observations, done):

        speed = observations[0]
        distance = observations[1]

        # Positive Reinforcement
        reward_distance = distance * 10.0
        # Negative Reinforcement for magnitude of speed
        reward_for_efective_movement = -1 * abs(speed)

        reward = reward_distance + reward_for_efective_movement

        rospy.loginfo("Reward_distance=" + str(reward_distance))
        rospy.loginfo("Reward_for_efective_movement= " + str(reward_for_efective_movement))

        return reward

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

    # RobotEnv methods
    # ----------------------------

    def _set_action(self, action):
        # TODO: This has to be adapted to the way the cube moves
        assert action.shape == (4,)
        action = action.copy()  # ensure that we don't change the action outside of this scope
        pos_ctrl, gripper_ctrl = action[:3], action[3]

        pos_ctrl *= 0.05  # limit maximum change in position
        rot_ctrl = [1., 0., 1., 0.]  # fixed rotation of the end effector, expressed as a quaternion
        gripper_ctrl = numpy.array([gripper_ctrl, gripper_ctrl])
        assert gripper_ctrl.shape == (2,)
        if self.block_gripper:
            gripper_ctrl = numpy.zeros_like(gripper_ctrl)
        action = numpy.concatenate([pos_ctrl, rot_ctrl, gripper_ctrl])

        # Apply action to simulation.
        utils.ctrl_set_action(self.sim, action)
        utils.mocap_set_action(self.sim, action)

    def _get_obs(self):
        # We convert from quaternions to euler
        orientation_list = [self.odom.pose.pose.orientation.x,
                            self.odom.pose.pose.orientation.y,
                            self.odom.pose.pose.orientation.z,
                            self.odom.pose.pose.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        # We get the distance from the origin
        start_position = Point()
        start_position.x = 0.0
        start_position.y = 0.0
        start_position.z = 0.0

        distance = self.get_distance_from_point(start_position,
                                                self.odom.pose.pose.position)

        cube_state = [
            round(self.joints.velocity[0], 1),
            round(distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(yaw, 1)
        ]

        return cube_state


    def _viewer_setup(self):
        body_id = self.sim.model.body_name2id('robot0:gripper_link')
        lookat = self.sim.data.body_xpos[body_id]
        for idx, value in enumerate(lookat):
            self.viewer.cam.lookat[idx] = value
        self.viewer.cam.distance = 2.5
        self.viewer.cam.azimuth = 132.
        self.viewer.cam.elevation = -14.

    def _render_callback(self):
        # Visualize target.
        sites_offset = (self.sim.data.site_xpos - self.sim.model.site_pos).copy()
        site_id = self.sim.model.site_name2id('target0')
        self.sim.model.site_pos[site_id] = self.goal - sites_offset[0]
        self.sim.forward()

    def _reset_sim(self):
        self.sim.set_state(self.initial_state)

        # Randomize start position of object.
        if self.has_object:
            object_xpos = self.initial_gripper_xpos[:2]
            while numpy.linalg.norm(object_xpos - self.initial_gripper_xpos[:2]) < 0.1:
                object_xpos = self.initial_gripper_xpos[:2] + self.numpy_random.uniform(-self.obj_range, self.obj_range, size=2)
            object_qpos = self.sim.data.get_joint_qpos('object0:joint')
            assert object_qpos.shape == (7,)
            object_qpos[:2] = object_xpos
            self.sim.data.set_joint_qpos('object0:joint', object_qpos)

        self.sim.forward()
        return True

    def _sample_goal(self):
        if self.has_object:
            goal = self.initial_gripper_xpos[:3] + self.numpy_random.uniform(-self.target_range, self.target_range, size=3)
            goal += self.target_offset
            goal[2] = self.height_offset
            if self.target_in_the_air and self.numpy_random.uniform() < 0.5:
                goal[2] += self.numpy_random.uniform(0, 0.45)
        else:
            goal = self.initial_gripper_xpos[:3] + self.numpy_random.uniform(-0.15, 0.15, size=3)
        return goal.copy()

    def _is_success(self, achieved_goal, desired_goal):
        d = goal_distance(achieved_goal, desired_goal)
        return (d < self.distance_threshold).astype(numpy.float32)

    def _is_done(self, observations):
        # Maximum distance to travel permited in meters from origin
        max_distance = 2.0

        if (observations[1] > max_distance):
            rospy.logerr("Cube Too Far==>" + str(observations[1]))
            done = True
        else:
            rospy.loginfo("Cube NOT Too Far==>" + str(observations[1]))
            done = False

        return done

    def _env_setup(self, initial_qpos):
        for name, value in initial_qpos.items():
            self.sim.data.set_joint_qpos(name, value)
        utils.reset_mocap_welds(self.sim)
        self.sim.forward()

        # Move end effector into position.
        gripper_target = numpy.array([-0.498, 0.005, -0.431 + self.gripper_extra_height]) + self.sim.data.get_site_xpos('robot0:grip')
        gripper_rotation = numpy.array([1., 0., 1., 0.])
        self.sim.data.set_mocap_pos('robot0:mocap', gripper_target)
        self.sim.data.set_mocap_quat('robot0:mocap', gripper_rotation)
        for _ in range(10):
            self.sim.step()

        # Extract information for sampling goals.
        self.initial_gripper_xpos = self.sim.data.get_site_xpos('robot0:grip').copy()
        if self.has_object:
            self.height_offset = self.sim.data.get_site_xpos('object0')[2]