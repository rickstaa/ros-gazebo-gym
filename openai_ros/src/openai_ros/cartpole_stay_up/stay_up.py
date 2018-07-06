from gym import utils
from openai_ros import cartpole_env
from gym.envs.registration import register
from gym import error, spaces
import rospy
import math
import numpy as np

# The path is __init__.py of openai_ros, where we import the MovingCubeOneDiskWalkEnv directly
register(
        id='CartPoleStayUp-v0',
        entry_point='openai_ros:CartPoleStayUpEnv',
        timestep_limit=1000,
    )

class CartPoleStayUpEnv(cartpole_env.CartPoleEnv):
    def __init__(self):
        
        self.get_params()
        
        self.action_space = spaces.Discrete(self.n_actions)
        high = np.array([
            2.5 * 2,
            np.finfo(np.float32).max,
            0.7 * 2,
            np.finfo(np.float32).max])
        self.observation_space = spaces.Box(-high, high)
        
        cartpole_env.CartPoleEnv.__init__(
            self, control_type=self.control_type
            )
            
    def get_params(self):
        #get configuration parameters
        self.n_actions = rospy.get_param('/cartpole_v0/n_actions')
        self.min_pole_angle = rospy.get_param('/cartpole_v0/min_pole_angle')
        self.max_pole_angle = rospy.get_param('/cartpole_v0/max_pole_angle')
        self.max_base_velocity = rospy.get_param('/cartpole_v0/max_base_velocity')
        self.min_base_pose_x = rospy.get_param('/cartpole_v0/min_base_pose_x')
        self.max_base_pose_x = rospy.get_param('/cartpole_v0/max_base_pose_x')
        self.pos_step = rospy.get_param('/cartpole_v0/pos_step')
        self.running_step = rospy.get_param('/cartpole_v0/running_step')
        self.init_pos = rospy.get_param('/cartpole_v0/init_pos')
        self.wait_time = rospy.get_param('/cartpole_v0/wait_time')
        self.control_type = rospy.get_param('/cartpole_v0/control_type')
        
    def _set_action(self, action):
        
        # Take action
        if action == 0: #LEFT
            rospy.loginfo("GO LEFT...")
            self.pos[0] -= self.pos_step
        elif action == 1: #RIGHT
            rospy.loginfo("GO RIGHT...")
            self.pos[0] += self.pos_step
        elif action == 2: #LEFT BIG
            rospy.loginfo("GO LEFT BIG...")
            self.pos[0] -= self.pos_step * 10
        elif action == 3: #RIGHT BIG
            rospy.loginfo("GO RIGHT BIG...")
            self.pos[0] += self.pos_step * 10
            
            
        # Apply action to simulation.
        rospy.loginfo("MOVING TO POS=="+str(self.pos))

        # 1st: unpause simulation
        rospy.logdebug("Unpause SIM...")
        self.gazebo.unpauseSim()

        self.move_joints(self.pos)
        rospy.logdebug("Wait for some time to execute movement, time="+str(self.running_step))
        rospy.sleep(self.running_step) #wait for some time
        rospy.logdebug("DONE Wait for some time to execute movement, time=" + str(self.running_step))

        # 3rd: pause simulation
        rospy.logdebug("Pause SIM...")
        self.gazebo.pauseSim()

    def _get_obs(self):
        
        data = self.joints
        #       base_postion                base_velocity              pole angle                 pole velocity
        obs = [round(data.position[1],1), round(data.velocity[1],1), round(data.position[0],1), round(data.velocity[0],1)]

        return obs
        
    def _is_done(self, observations):
        done = False
        data = self.joints
        #       base_postion            base_velocity           pole angle              pole velocity
        #state = [round(data.position[1],1), round(data.velocity[1],1), round(data.position[0],1), round(data.velocity[0],1)]
        #   pole angle  pole velocity
        rospy.loginfo("BASEPOSITION=="+str(observations[0]))
        rospy.loginfo("POLE ANGLE==" + str(observations[2]))
        if (self.min_base_pose_x >= observations[0] or observations[0] >= self.max_base_pose_x): #check if the base is still within the ranges of (-2, 2)
            rospy.logerr("Base Outside Limits==>min="+str(self.min_base_pose_x)+",pos="+str(observations[0])+",max="+str(self.max_base_pose_x))
            done = True
        if (self.min_pole_angle >= observations[2] or observations[2] >= self.max_pole_angle): #check if pole has toppled over
            rospy.logerr(
                "Pole Angle Outside Limits==>min=" + str(self.min_pole_angle) + ",pos=" + str(observations[2]) + ",max=" + str(
                    self.max_pole_angle))
            done = True
            
        return done
        
    def _compute_reward(self, observations, done):

        """
        Gives more points for staying upright, gets data from given observations to avoid
        having different data than other previous functions
        :return:reward
        """
        """
        pole_angle = observations[2]
        pole_vel = observations[3]

        rospy.loginfo("pole_angle for reward==>" + str(pole_angle))
        delta = 0.7 - abs(pole_angle)
        reward_pole_angle = math.exp(delta*10)
        #reward_pole_angle = math.exp(delta*2)

        # If we are moving to the left and the pole is falling left is Bad
        rospy.loginfo("pole_vel==>" + str(pole_vel))
        pole_vel_sign = np.sign(pole_vel)
        pole_angle_sign = np.sign(pole_angle)
        rospy.loginfo("pole_vel sign==>" + str(pole_vel_sign))
        rospy.loginfo("pole_angle sign==>" + str(pole_angle_sign))

        # We want inverted signs for the speeds. We multiply by -1 to make minus positive.
        # global_sign + = GOOD, global_sign - = BAD
        base_reward = 500
        #base_reward = 100
        if pole_vel != 0:
            global_sign = pole_angle_sign * pole_vel_sign * -1
            reward_for_efective_movement = base_reward * global_sign
        else:
            # Is a particular case. If it doesnt move then its good also
            reward_for_efective_movement = base_reward

        reward = reward_pole_angle + reward_for_efective_movement

        rospy.loginfo("reward==>" + str(reward)+"= r_pole_angle="+str(reward_pole_angle)+",r_movement= "+str(reward_for_efective_movement))
        """
        
        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None:
            # Pole just fell!
            self.steps_beyond_done = 0
            reward = 1.0
        else:
            if self.steps_beyond_done == 0:
                logger.warning("You are calling 'step()' even though this environment has already returned done = True. You should always call 'reset()' once you receive 'done = True' -- any further steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0
        
        return reward
        
    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        pass
