import rospy
import tf
from moveit_msgs.msg import PlanningScene
from openai_ros.core import ROSLauncher
from openai_ros.robot_gazebo_env import RobotGazeboEnv
from sensor_msgs.msg import Imu, JointState


class ShadowTcEnv(RobotGazeboEnv):
    """Superclass for all ShadowTcEnv environments."""

    def __init__(self, workspace_path):
        """
        Initializes a new ShadowTcEnv environment.

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
            * /imu/data
            * /joint_states


        Actuators Topic List:
            * As actuator we will use a class SmartGrasper to interface.
              We use smart_grasping_sandbox smart_grasper.py, to move and get the pose
              of the ball and the tool tip.

        Args:
            workspace_path (str, optional): The path of the workspace in which the
                shadow_env_gazebo package should be found. Defaults to ``None``.
        """
        rospy.logdebug("Start ShadowTcEnv INIT...")
        # Variables that we give through the constructor.
        # None in this case

        # We launch the ROSlaunch that spawns the robot into the world
        ROSLauncher(
            package_name="shadow_gazebo",
            launch_file_name="put_shadow_in_world.launch",
            workspace_path=workspace_path,
        )

        # Internal Vars
        # Doesn't have any accessibles
        self.controllers_list = []

        # It doesn't use namespace
        self.robot_name_space = ""

        # We launch the init function of the parent class
        # RobotGazeboEnv
        super(ShadowTcEnv, self).__init__(
            controllers_list=self.controllers_list,
            robot_name_space=self.robot_name_space,
            reset_controls=False,
            reset_world_or_sim="NO_RESET_SIM",
        )

        rospy.logdebug("ShadowTcEnv unpause...")
        self.gazebo.unpause_sim()
        # self.controllers_object.reset_controllers()

        self._check_all_systems_ready()

        rospy.Subscriber("/imu/data", Imu, self._imu_callback)
        rospy.Subscriber("/joint_states", JointState, self._joints_state_callback)
        # rospy.Subscriber(
        #     "/planning_scene", PlanningScene, self._planning_scene_callback
        # )

        self._setup_smart_grasper()

        self.gazebo.pause_sim()

        rospy.logdebug("Finished ShadowTcEnv INIT...")

    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        rospy.logdebug("ShadowTcEnv check_all_systems_ready...")
        self._check_all_sensors_ready()
        rospy.logdebug("END ShadowTcEnv _check_all_systems_ready...")
        return True

    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        self._check_imu_ready()
        self._check_joint_states_ready()
        # self._check_planning_scene_ready()

        rospy.logdebug("ALL SENSORS READY")

    def _check_imu_ready(self):
        self.imu = None
        rospy.logdebug("Waiting for /imu/data to be READY...")
        while self.imu is None and not rospy.is_shutdown():
            try:
                self.imu = rospy.wait_for_message("/imu/data", Imu, timeout=5.0)
                rospy.logdebug("Current/imu/data READY=>")
            except Exception:
                rospy.logerr(
                    "Current /imu/data not ready yet, retrying for getting imu"
                )

        return self.imu

    def _check_joint_states_ready(self):
        self.joint_states = None
        rospy.logdebug("Waiting for /joint_states to be READY...")
        while self.joint_states is None and not rospy.is_shutdown():
            try:
                self.joint_states = rospy.wait_for_message(
                    "/joint_states", JointState, timeout=1.0
                )
                rospy.logdebug("Current /joint_states READY=>")
            except Exception:
                rospy.logerr(
                    "Current /joint_states not ready yet, retrying for getting "
                    "joint_states."
                )
        return self.joint_states

    def _check_planning_scene_ready(self):
        self.planning_scene = None
        rospy.logdebug("Waiting for /planning_scene to be READY...")
        while self.planning_scene is None and not rospy.is_shutdown():
            try:
                self.planning_scene = rospy.wait_for_message(
                    "/planning_scene", PlanningScene, timeout=1.0
                )
                rospy.logdebug("Current /planning_scene READY=>")
            except Exception:
                rospy.logerr(
                    "Current /planning_scene not ready yet, retrying for getting "
                    "planning_scene."
                )
        return self.planning_scene

    def _imu_callback(self, data):
        self.imu = data

    def _joints_state_callback(self, data):
        self.joint_states = data

    def _planning_scene_callback(self, data):
        self.planning_scene = data

    def _setup_tf_listener(self):
        """
        Set ups the TF listener for getting the transforms you ask for.
        """
        self.listener = tf.TransformListener()

    def _setup_smart_grasper(self):
        """
        Setup of the movement system.
        """
        rospy.logdebug("START _setup_smart_grasper")
        # We need to tell it to not start a node
        from smart_grasping_sandbox.smart_grasper import SmartGrasper

        self.sgs = SmartGrasper(init_ros_node=False)
        rospy.logdebug("END _setup_smart_grasper")

    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its initial pose."""
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given."""
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation."""
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given."""
        raise NotImplementedError()

    # Methods that the TrainingEnvironment will need.
    # ----------------------------

    def open_hand(self):
        """
        When called it opens robots hand
        """
        self.sgs.open_hand()

    def close_hand(self):
        """
        When called it closes robots hand
        """
        self.sgs.close_hand()

    def get_ball_pose(self):
        """
        Get Ball Pose
        return: Ball Pose in the World frame
        We unpause and pause the simulation because this calss is a service call.
        This means that if the simulation is NOT
        running it won't get the Ball information of position.
        """
        rospy.logdebug("START get_ball_pose ==>")
        self.gazebo.unpause_sim()
        ball_pose = self.sgs.get_object_pose()
        self.gazebo.pause_sim()
        rospy.logdebug("ball_pose ==>" + str(ball_pose))
        rospy.logdebug("STOP get_ball_pose ==>")

        return ball_pose

    def get_tip_pose(self):
        """
        Returns the pose of the tip of the TCP
        We unpause and pause the simulation because this calss is a service call.
        This means that if the simulation is NOT
        running it won't get the TCP information of position.
        """
        rospy.logdebug("START get_tip_pose ==>")
        self.gazebo.unpause_sim()
        tcp_pose = self.sgs.get_tip_pose()
        self.gazebo.pause_sim()
        rospy.logdebug("END get_tip_pose ==>")
        return tcp_pose

    def move_tcp_world_frame(self, desired_pose):
        """
        Moves the Tool tip TCP to the pose given. Its relative pose to world frame.

        Args:
            desired_pose: Pose where you want the TCP to move next
        """
        self.sgs.move_tip_absolute(desired_pose)

    def move_tip(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Moves that increment of XYZ RPY in the world frame
        Only state the increment of the variable you want, the rest will
        not increment due to the default values
        """
        self.sgs.move_tip(x, y, z, roll, pitch, yaw)

    def send_movement_command(self, command, duration=0.2):
        """
        Send a dictionnary of joint targets to the arm and hand directly.
        To get the available joints names: rostopic echo /joint_states/name -n1
        [H1_F1J1, H1_F1J2, H1_F1J3, H1_F2J1, H1_F2J2, H1_F2J3, H1_F3J1, H1_F3J2,
        H1_F3J3, elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint,
        wrist_2_joint, wrist_3_joint]

        Args:
            command: a dictionnary of joint names associated with a target:
                {"H1_F1J1": -1.0, "shoulder_pan_joint": 1.0}
            duration: the amount of time it will take to get there in seconds. Needs
                to be bigger than 0.0
        """
        self.sgs.send_command(command, duration)

    def set_fingers_colision(self, activate=False):
        """
        It activates or deactivates the finger collisions.
        It also will triguer the publish into the planning_scene the collisions.
        We puase and unpause for the smae exact reason as the get TCP and get ball pos.
        Being a service, untill the simulation is unpaused it won't get response.
        """
        rospy.logdebug("START get_fingers_colision")
        self.sgs.check_fingers_collisions(activate)
        rospy.logdebug("END get_fingers_colision")

    def get_fingers_colision(self, object_collision_name):
        """
        Returns the collision of the three fingers
        object_collision_name: Here yo ustate the name of the model to check collision
        with fingers.
        Objects in sim: cricket_ball__link, drill__link
        """
        self.gazebo.unpause_sim()
        self.set_fingers_colision(True)
        planning_scene = self._check_planning_scene_ready()
        self.gazebo.pause_sim()

        objects_scene = planning_scene.allowed_collision_matrix.entry_names
        colissions_matrix = planning_scene.allowed_collision_matrix.entry_values

        # We look for the Ball object model name in the objects sceen list and get the
        # index
        object_collision_name_index = objects_scene.index(object_collision_name)

        Finger_Links_Names = [
            "H1_F1_base_link",
            "H1_F1_link_1",
            "H1_F1_link_2",
            "H1_F1_palm_link",
            "H1_F1_tip",
            "H1_F2_base_link",
            "H1_F2_link_1",
            "H1_F2_link_2",
            "H1_F2_palm_link",
            "H1_F2_tip",
            "H1_F3_base_link",
            "H1_F3_link_1",
            "H1_F3_link_2",
            "H1_F3_palm_link",
            "H1_F3_tip",
        ]

        # We get all the index of the model links that are part of the fingers
        # We separate by finguer to afterwards be easy to detect that there is contact
        # in all of the finguers
        finger1_indices = [
            i for i, var in enumerate(Finger_Links_Names) if "H1_F1" in var
        ]
        finger2_indices = [
            i for i, var in enumerate(Finger_Links_Names) if "H1_F2" in var
        ]
        finger3_indices = [
            i for i, var in enumerate(Finger_Links_Names) if "H1_F3" in var
        ]

        # Now we search in the entry_value corresponding to the object to check the
        # collision with all the rest of objects.
        object_collision_array = colissions_matrix[object_collision_name_index].enabled

        # Is there a collision with Finguer1
        f1_collision = False
        for finger_index in finger1_indices:
            if object_collision_array[finger_index]:
                f1_collision = True
                break

        # Is there a collision with Finguer2
        f2_collision = False
        for finger_index in finger2_indices:
            if object_collision_array[finger_index]:
                f2_collision = True
                break

        # Is there a collision with Finguer3
        f3_collision = False
        for finger_index in finger3_indices:
            if object_collision_array[finger_index]:
                f3_collision = True
                break

        finger_collision_dict = {
            "f1": f1_collision,
            "f2": f2_collision,
            "f3": f3_collision,
        }

        return finger_collision_dict

    def reset_scene(self):
        """
        Restarts the simulation and world objects
        """
        self.sgs.reset_world()

    def get_imu(self):
        return self.imu

    def get_joint_states(self):
        return self.joint_states
