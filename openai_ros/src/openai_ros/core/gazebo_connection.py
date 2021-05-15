#!/usr/bin/env python3
"""Contains a small python API class that makes it easier to interact with the Gazebo
simulator.
"""

import rospy
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from std_srvs.srv import Empty


class GazeboConnection:
    """Class that contains several methods that can be used to interact with the Gazebo
    simulation.

    Attributes:
        pause_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS service that
            pauses the gazebo simulator.
        unpause_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS service that
            un-pauses the gazebo simulator.
        reset_simulation_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS
            service that resets the gazebo simulator.
        reset_world_proxy (:obj:`rospy.impl.tcpros_service.ServiceProxy`): ROS service
            that resets the gazebo world.
    """

    def __init__(self, start_init_physics_parameters, reset_world_or_sim, max_retry=20):
        """Initiate the GazeboConnection instance.

        Args:
            start_init_physics_parameters (bool, optional): Wether you want to
                initialize the simulation parameters. Defaults to True.
            reset_world_or_sim (str, optional): Wether you want to reset the whole
                simulation "SIMULATION" at startup or only the world "WORLD" (object
                positions). Defaults to "SIMULATION".
            max_retry (int, optional): How many times a command to the simulator is
                retried before giving up. Defaults to 20.
        """
        self.pause_proxy = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause_proxy = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.reset_simulation_proxy = rospy.ServiceProxy(
            "/gazebo/reset_simulation", Empty
        )
        self.reset_world_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self._max_retry = max_retry

        # Setup the physics properties controle system
        service_name = "/gazebo/set_physics_properties"
        rospy.logdebug("Waiting for service " + str(service_name))
        rospy.wait_for_service(service_name)
        rospy.logdebug("Service Found " + str(service_name))
        self.set_physics = rospy.ServiceProxy(service_name, SetPhysicsProperties)
        self.start_init_physics_parameters = start_init_physics_parameters
        self.reset_world_or_sim = reset_world_or_sim
        self.init_values()

        # We always pause the simulation, important for legged robots learning
        self.pause_sim()

    def pause_sim(self):
        """Pause the simulation."""
        rospy.logdebug("PAUSING service found...")
        paused_done = False
        counter = 0
        while not paused_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    rospy.logdebug("PAUSING service calling...")
                    self.pause_proxy()
                    paused_done = True
                    rospy.logdebug("PAUSING service calling...DONE")
                except rospy.ServiceException:
                    counter += 1
                    rospy.logerr("/gazebo/pause_physics service call failed")
            else:
                error_message = (
                    "Maximum retries done"
                    + str(self._max_retry)
                    + ", please check Gazebo pause service"
                )
                rospy.logerr(error_message)
                assert False, error_message

        rospy.logdebug("PAUSING finished")

    def unpause_sim(self):
        """Unpauses the simulation."""
        rospy.logdebug("UNPAUSING service found...")
        unpaused_done = False
        counter = 0
        while not unpaused_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    rospy.logdebug("UNPAUSING service calling...")
                    self.unpause_proxy()
                    unpaused_done = True
                    rospy.logdebug("UNPAUSING service calling...DONE")
                except rospy.ServiceException:
                    counter += 1
                    rospy.logerr(
                        "/gazebo/unpause_physics service call failed...Retrying "
                        + str(counter)
                    )
            else:
                error_message = (
                    "Maximum retries done"
                    + str(self._max_retry)
                    + ", please check Gazebo unpause service"
                )
                rospy.logerr(error_message)
                assert False, error_message

        rospy.logdebug("UNPAUSING finished")

    def _reset_simulation(self):
        """Calls the ROS reset simulation service."""
        rospy.wait_for_service("/gazebo/reset_simulation")
        try:
            self.reset_simulation_proxy()
        except rospy.ServiceException:
            print("/gazebo/reset_simulation service call failed")

    def _reset_world(self):
        """Resets the world (NOT THE WHOLE SIMULATION)."""
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_world_proxy()
        except rospy.ServiceException:
            print("/gazebo/reset_world service call failed")

    def reset_sim(self):
        """Reset the simulation or the world.

        .. note::
            Implemented like this since in some simulations, when reset the simulation
            the systems that work with TF break. In this case we ONLY resets the object
            position, not the entire simulation.
        """
        if self.reset_world_or_sim == "SIMULATION":
            rospy.logerr("SIMULATION RESET")
            self._reset_simulation()
        elif self.reset_world_or_sim == "WORLD":
            rospy.logerr("WORLD RESET")
            self._reset_world()
        elif self.reset_world_or_sim == "NO_RESET_SIM":
            rospy.logerr("NO RESET SIMULATION SELECTED")
        else:
            rospy.logerr("WRONG Reset Option:" + str(self.reset_world_or_sim))

    def _init_physics_parameters(self):
        """Initialise the physics parameters of the simulation, like gravity,
        friction coefficients and so on.
        """
        self._time_step = Float64(0.001)
        self._max_update_rate = Float64(1000.0)

        self._gravity = Vector3()
        self._gravity.x = 0.0
        self._gravity.y = 0.0
        self._gravity.z = -9.81

        self._ode_config = ODEPhysics()
        self._ode_config.auto_disable_bodies = False
        self._ode_config.sor_pgs_precon_iters = 0
        self._ode_config.sor_pgs_iters = 50
        self._ode_config.sor_pgs_w = 1.3
        self._ode_config.sor_pgs_rms_error_tol = 0.0
        self._ode_config.contact_surface_layer = 0.001
        self._ode_config.contact_max_correcting_vel = 0.0
        self._ode_config.cfm = 0.0
        self._ode_config.erp = 0.2
        self._ode_config.max_contacts = 20

        self._update_gravity_call()

    def init_values(self):
        """Sets the initial simulator parameter values."""
        self.reset_sim()
        if self.start_init_physics_parameters:
            rospy.logdebug("Initialising simulation Physics Parameters")
            self._init_physics_parameters()
        else:
            rospy.logerr("NOT Initialising simulation Physics Parameters")

    def _update_gravity_call(self):
        """Updates the simulator gravity property."""
        self.pause_sim()

        # Create physic change message
        set_physics_request = SetPhysicsPropertiesRequest()
        set_physics_request.time_step = self._time_step.data
        set_physics_request.max_update_rate = self._max_update_rate.data
        set_physics_request.gravity = self._gravity
        set_physics_request.ode_config = self._ode_config
        rospy.logdebug(str(set_physics_request.gravity))

        # Send physic change request
        result = self.set_physics(set_physics_request)
        rospy.logdebug(
            "Gravity Update Result=="
            + str(result.success)
            + ",message=="
            + str(result.status_message)
        )

        self.unpause_sim()

    def change_gravity(self, x, y, z):
        """Changes the gravity vector

        Args:
            x (float): Gravity vector x coordinate.
            y (float): Gravity vector y coordinate.
            z (float): Gravity vector z coordinate.
        """
        self._gravity.x = x
        self._gravity.y = y
        self._gravity.z = z
        self._update_gravity_call()
