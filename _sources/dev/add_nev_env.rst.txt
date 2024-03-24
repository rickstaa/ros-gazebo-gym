.. _add_new_env:

====================
Add new environments
====================

Welcome to the guide on adding new environments to the :ros-gazebo-gym:`ros_gazebo_gym <>` project, an inclusive open-source initiative that invites contributions from the community. Prior to making your
own contribution, we strongly encourage a thorough review of the :ros-gazebo-gym:`contributing guidelines <blob/noetic/CONTRIBUTING.md>`. Below, we provide a concise and structured guide to assist
you in introducing a new ROS gazebo gymnasium environment. For any queries that arise, please feel free to open an `issue`_ to seek further clarification.

.. _issue: https://github.com/rickstaa/ros-gazebo-gym/issues

Creating task and robot Environments
------------------------------------

General instructions
~~~~~~~~~~~~~~~~~~~~

To develop a new environment, the :ros-gazebo-gym:`ros_gazebo_gym <>` package provides two templates to create ROS gymnasium environment wrappers:

    * ``template_my_robot_env.py``: This template constructs the robot environment, responsible for sensor data reading and actuator control.
    * ``template_my_task_env.py``: This template builds the task environment, responsible for initial environment setup, task reward computation, and episode completion checks.

To embark on creating a new environment, establish a new folder for your robot within the ``ros_gazebo_gym/task_envs`` directory. Copy the ``template_my_task_env.py`` into this folder and rename it
accordingly. Then, duplicate the ``template_my_robot_env.py`` file into the ``ros_gazebo_gym/robot_envs`` directory and provide it a new name. As you proceed, address the ``TODOS`` indicated in these templates.
It's crucial that the robot environment inherits from either the :class:`~ros_gazebo_gym.robot_gazebo_env.RobotGazeboEnv` or :class:`~ros_gazebo_gym.robot_gazebo_goal_env.RobotGazeboGoalEnv` classes.

.. Note::

    For a ``dictionary`` format for observations (as seen in :gymnasium-robotics:`gymnasium Fetch environments <envs/fetch/>`), use the :class:`~ros_gazebo_gym.robot_gazebo_goal_env.RobotGazeboGoalEnv`
    environment.For a simple array observation format, you can inherit from the :class:`~ros_gazebo_gym.robot_gazebo_env.RobotGazeboEnv` class.

A variety of helper classes and methods are accessible to assist in creating your ROS gymnasium environment:

    * :class:`~ros_gazebo_gym.core.controllers_connection.ControllersConnection`: Manages resetting, switching, and loading ROS controllers.
    * :class:`~ros_gazebo_gym.core.gazebo_connection.GazeboConnection`: Facilitates interaction with the Gazebo simulation, offering access to model and link information, pause/unpause functionality,
      model spawning, and simulation resetting.
    * :class:`~ros_gazebo_gym.core.lazy_importer.LazyImporter`: Lazily imports ROS python packages to avoid import errors.
    * :class:`~ros_gazebo_gym.core.ros_launcher.ROSLauncher`: Launches ROS launch files in a safe subprocess manner.
    * :meth:`~ros_gazebo_gym.core.ros_launcher.ROSLauncher.initialize`: Ensures the presence of a running ROS master and Python script-initialized ROS environment.
    * :meth:`~ros_gazebo_gym.core.helpers.load_ros_params_from_yaml`: Imports ROS parameters from a ``yaml`` file.
    * :mod:`~ros_gazebo_gym.core.helpers`: Contains a variety of helper methods for ROS gazebo gymnasium environments.

Refer to the :ref:`Python API documentation <python_api>` for a comprehensive overview of all functions and classes within the :ros-gazebo-gym:`ros_gazebo_gym <>` package.

Recommendations
~~~~~~~~~~~~~~~

Package structure
^^^^^^^^^^^^^^^^^

When constructing your environment, it's advisable to house ROS launch files within a distinct ROS package. This separation ensures the cleanliness of the :ros-gazebo-gym:`ros_gazebo_gym <>` package. Incorporate
this dependency into the :ros-gazebo-gym:`ros_gazebo_gym/cfg/rosdep.yaml <blob/noetic/src/ros_gazebo_gym/cfg/rosdep.yaml>` file:

.. literalinclude:: ../../../src/ros_gazebo_gym/cfg/rosdep.yaml
   :language: python
   :linenos:

After incorporating dependencies, utilize the :class:`~ros_gazebo_gym.core.ros_launcher.ROSLauncher` to execute your launch files. This class guarantees that all prerequisites for your environment are installed
and constructed. For an example, refer to the :class:`~ros_gazebo_gym.robot_envs.panda_env.PandaEnv` and the :panda-gazebo:`panda_gazebo <>` package.

Environment configuration
^^^^^^^^^^^^^^^^^^^^^^^^^

It's recommended to store task environment configuration parameters in a dedicated configuration file located within a ``config`` folder in your task environment directory. These parameters can be loaded onto the
ROS parameter server using the :meth:`ros_gazebo_gym.core.helpers.load_ros_params_from_yaml` method. This enables their use in both the Task and Robot classes. An example can be found in the
:class:`~ros_gazebo_gym.task_envs.panda.panda_reach.PandaReachEnv` environment. The configuration parameters are stored in the :ros-gazebo-gym:`panda_reach.yaml <blob/noetic/src/ros_gazebo_gym/task_envs/panda/config/panda_reach.yaml>`
file:

.. literalinclude:: ../../../src/ros_gazebo_gym/task_envs/panda/config/panda_reach.yaml
   :language: python
   :linenos:
   :lines: -42

These parameters are loaded onto the ROS parameter server within the :meth:`ros_gazebo_gym.task_envs.panda.panda_reach.PandaReachEnv.__init__`
method using the :meth:`ros_gazebo_gym.task_envs.panda.panda_reach.PandaReachEnv._get_params` method.

Registering the environment
---------------------------

Upon creating your environment, it must be registered within the :ros-gazebo-gym:`ros_gazebo_gym <>` package to be discoverable by :gymnasium:`gymnasium <>`. Achieve this by incorporating information about your environment within
the :obj:`ros_gazebo_gym.task_envs.task_envs_list` configuration file:

.. literalinclude:: ../../../src/ros_gazebo_gym/task_envs/task_envs_list.py
   :language: python
   :linenos:
   :lines: 18-26

Each environment entry should include ``module``, ``max_steps``, and ``reward_threshold`` attributes. Following this addition, the :ros-gazebo-gym:`ros_gazebo_gym <>` package will facilitate the environment's registration
under the gymnasium namespace. This will enable environment creation using the :func:`gym.make` method.
