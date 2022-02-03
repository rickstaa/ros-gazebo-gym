.. _add_new_env:

===================
Add new environment
===================

The :ros_gazebo_gym:`ros_gazebo_gym <>` is a open-source project that welcomes all contributions. Before contributing we kindly ask
to review the :ros_gazebo_gym:`contributing guidelines <contributing.md>`. Below you will find a quick guide on adding a new ROS gazebo gym environment. Feel free to open an :issues:`issue <>`
if you have any questions.

Create the Task and Robot environments
--------------------------------------

General instructions
~~~~~~~~~~~~~~~~~~~~

The :ros_gazebo_gym:`ros_gazebo_gym <>` package contains two templates to create your own ROS gym environment wrapper:

    * ``template_my_robot_env.py``: This is used to create the robot environment responsible for reading sensor data and controlling actuators.
    * ``template_my_task_env.py``: This is used to create the task environment responsible for setting up the initial environment state, calculating the task reward, and checking whether an episode is done.

To start creating a new environment, create a new folder for your robot inside the ``ros_gazebo_gym/task_envs`` folder,
copy the ``template_my_task_env.py`` into this folder and rename it. Next, copy the ``template_my_robot_env.py`` file into
the ``ros_gazebo_gym/robot_envs`` folder and rename it. Following, you can complete the ``TODOS`` found in these templates.
While doing so, please make sure that the robot environment inherits from the :class:`~ros_gazebo_gym.robot_gazebo_env.RobotGazeboEnv`
or :class:`~ros_gazebo_gym.robot_gazebo_goal_env.RobotGazeboGoalEnv` classes.

.. Note::

    The :class:`~ros_gazebo_gym.robot_gazebo_goal_env.RobotGazeboGoalEnv` environment should be used if you want the observations to be
    a ``dictionary`` (see the `openai Gym Fetch environments <https://gym.openai.com/envs/FetchPush-v0/>`_). If you want the
    observations to be a simple array, you can inherit from the :class:`~ros_gazebo_gym.robot_gazebo_env.RobotGazeboEnv` class.

There are several helper classes and methods available that help you create your ROS gym environment:

    * :class:`~ros_gazebo_gym.core.controllers_connection.ControllersConnection`: Class used to reset/switch and load ROS controllers.
    * :class:`~ros_gazebo_gym.core.gazebo_connection.GazeboConnection`: Class used to interact with the Gazebo simulation. It can retrieve information about the models and links inside the simulation, pause/unpause, spawn models and reset the simulation.
    * :class:`~ros_gazebo_gym.core.ros_launcher.ROSLauncher`: A class that can be used to launch ROS launch files as subprocesses in a fail-safe way.
    * :meth:`~ros_gazebo_gym.core.ros_launcher.ROSLauncher.initialize`: Makes sure that a ROS master is running and that the python script initialized ROS.
    * :meth:`~ros_gazebo_gym.core.helpers.load_ros_params_from_yaml`: Can be used to load ROS parameters from a ``yaml`` file.

For all the functions and classes inside the :ros_gazebo_gym:`ros_gazebo_gym <>` package please see the :ref:`api`.

Recommendations
~~~~~~~~~~~~~~~

Package structure
^^^^^^^^^^^^^^^^^

When building your environment, you are recommended to host any ROS launch files inside a separate ROS package. This is done to keep the :ros_gazebo_gym:`ros_gazebo_gym <>` package clean.
You can then make the :ros_gazebo_gym:`ros_gazebo_gym <>` package aware of this dependency by adding it in the :ros_gazebo_gym:`ros_gazebo_gym/cfg/rosdep.yaml <cfg/rosdep.yaml>` file:

.. literalinclude:: ../../../src/ros_gazebo_gym/cfg/rosdep.yaml
   :language: python
   :linenos:

After adding your dependencies, you can execute your launch files using the aforementioned :class:`~ros_gazebo_gym.core.ros_launcher.ROSLauncher`. This class will then make sure that all the dependencies
for your environment are installed and built. See the :class:`~ros_gazebo_gym.robot_envs.panda_env.PandaEnv` and the :panda_gazebo:`panda_gazebo <>` package for an example.

Environment configuration
^^^^^^^^^^^^^^^^^^^^^^^^^

You are advised to put task environment configuration parameters inside a separate configuration file. This file should be contained inside a ``config`` folder inside your task environment folder. The configuration parameters
in this file can then be loaded onto the ROS parameter server using the :meth:`ros_gazebo_gym.core.helpers.load_ros_params_from_yaml` method. This will make them available for use in both the Task end Robot class. A example
can be seen in the :class:`~ros_gazebo_gym.task_envs.panda.panda_reach.PandaReachEnv` environment. For this environment the configuration parameters are stored inside the :ros_gazebo_gym:`panda_reach.yaml <src/ros_gazebo_gym/task_envs/panda/config/panda_reach.yaml>`
file:

.. literalinclude:: ../../../src/ros_gazebo_gym/task_envs/panda/config/panda_reach.yaml
   :language: python
   :linenos:
   :lines: -42

The parameters in this file are loaded on the ROS parameter server inside the :meth:`ros_gazebo_gym.task_envs.panda.panda_reach.PandaReachEnv.__init__`
method via the :meth:`ros_gazebo_gym.task_envs.panda.panda_reach.PandaReachEnv._get_params` method.

Register the environment
------------------------

After you created your environment you have to register it under the :ros_gazebo_gym:`ros_gazebo_gym <>` package for `Openai gym`_ to find it. This can be done by
adding information about your environment inside the :obj:`ros_gazebo_gym.task_envs.task_envs_list` configuration file:

.. literalinclude:: ../../../src/ros_gazebo_gym/task_envs/task_envs_list.py
   :language: python
   :linenos:
   :lines: 18-26

Each environment should contain a ``module``, ``max_steps`` and ``reward_threshold`` field. After adding this information, the :ros_gazebo_gym:`ros_gazebo_gym <>` package
will ensure the environment is registered under the Openai Gym namespace and can be created using the :meth:`gym.make` method.

.. _`Openai gym`: https://gym.openai.com/
