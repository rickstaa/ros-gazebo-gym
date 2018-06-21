import rospy
from openai_gazebo import cube_single_disk_env
from gym.envs.registration import register
from geometry_msgs.msg import Point

# The path is __init__.py of openai_gazebo, where we import the MovingCubeOneDiskWalkEnv directly
register(
        id='MovingCubeOneDiskWalk-v0',
        entry_point='openai_gazebo:MovingCubeOneDiskWalkEnv',
        timestep_limit=1000,
    )

class MovingCubeOneDiskWalkEnv(cube_single_disk_env.CubeSingleDiskEnv):
    def __init__(self):
        number_actions = rospy.get_param('/moving_cube/n_actions')
        # Variables that we retrieve through the param server, loded when launch training launch.
        self.roll_speed_fixed_value = rospy.get_param('/moving_cube/roll_speed_fixed_value')
        self.max_distance = rospy.get_param('/moving_cube/max_distance')

        self.start_point = Point()
        self.start_point.x = rospy.get_param("/moving_cube/init_cube_pose/x")
        self.start_point.y = rospy.get_param("/moving_cube/init_cube_pose/y")
        self.start_point.z = rospy.get_param("/moving_cube/init_cube_pose/z")

        self.end_episode_points = rospy.get_param("/moving_cube/end_episode_points")

        self.init_roll_vel = rospy.get_param("/moving_cube/init_roll_vel")

        # Here we will add any init functions prior to starting the CubeSingleDiskEnv
        super(MovingCubeOneDiskWalkEnv, self).__init__(number_actions, self.init_roll_vel)

    def _set_action(self, action):

        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        roll_turn_speed = None
        if action == 1:# Move Speed Wheel Forwards
            roll_turn_speed = self.roll_speed_fixed_value
        elif action == 2:# Move Speed Wheel Backwards
            roll_turn_speed = self.roll_speed_fixed_value
        elif action == 3:# Stop Speed Wheel
            roll_turn_speed = 0.0

        # We tell the OneDiskCube to spin the RollDisk at the selected speed
        self.move_joints(roll_turn_speed)

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        CubeSingleDiskEnv API DOCS
        :return:
        """

        # We get the orientation of the cube in RPY
        roll, pitch, yaw = self.get_orientation_euler()

        # We get the distance from the origin
        distance = self.get_distance_from_start_point(self.start_point)

        # We get the current speed of the Roll Disk
        current_roll_vel = self.get_roll_velocity()

        cube_observations = [
            round(current_roll_vel, 1),
            round(distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(yaw, 1)
        ]

        return cube_observations

    def _is_done(self, observations):
        # Maximum distance to travel permited in meters from origin
        max_distance = 2.0

        if observations[1] > self.max_distance:
            rospy.logerr("Cube Too Far==>" + str(observations[1]))
            done = True
        else:
            rospy.loginfo("Cube NOT Too Far==>" + str(observations[1]))
            done = False

        return done

    def _compute_reward(self, observations, done):

        if not done:
            speed = observations[0]
            distance = observations[1]

            # Positive Reinforcement
            reward_distance = distance * 10.0
            # Negative Reinforcement for magnitude of speed
            reward_for_efective_movement = -1 * abs(speed)

            reward = reward_distance + reward_for_efective_movement

            rospy.loginfo("Reward_distance=" + str(reward_distance))
            rospy.loginfo("Reward_for_efective_movement= " + str(reward_for_efective_movement))
        else:
            reward = self.end_episode_points

        return reward

