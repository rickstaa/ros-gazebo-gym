=========================================
Welcome to ros_gazebo_gym's documentation
=========================================

.. image:: /images/ros_gazebo_gym.png
   :alt: ROS Gazebo Gym

Welcome to the ROS Gazebo Gym framework! This framework provides all the tools necessary to create robot environments based on :ros:`ROS <>`, :gazebo:`Gazebo <>` and :gymnasium:`gymnasium <>`, and to train reinforcement
learning (RL) algorithms in these environments. It also contains pre-made environments for testing and benchmarking RL algorithms. 

RL algorithms have achieved impressive results in `games and simulations`_, but translating these results to real-world robots is challenging due to safety
and time constraints. Simulations are often used to train RL algorithms for real-world robots, but :gymnasium:`gymnasium <>`, often used for developing and comparing RL algorithms, is not
directly compatible with the simulated environments used in robotics research. The :ros-gazebo-gym:`ROS Gazebo Gym <>` framework provides a way to translate ROS :gazebo:`Gazebo <>` simulations into
gymnasium environments, focusing on delivering real-world-ready solutions. This means that algorithms trained in simulation can be readily applied to real robots. We aim to create a
common ground for people who use RL with real robots and accelerate research in this area.

.. _`games and simulations`: https://arxiv.org/abs/1912.06680

Package structure
=================

The structure of this package was based on the :openai_ros:`openai_ros <>` package created by `the construct`_. As a result, each ROS :gazebo:`Gazebo <>` gymnasium environment is
divided into three classes: a **task** environment, a **robot** environment and a **gazebo** environment.

.. image:: /images/ros_gazebo_gym_diagram.png
   :alt: ROS Gazebo Gym Structure Diagram

Using this compartmentalised structure makes the code base easier to understand and is easier to extend. Each of these classes is responsible for one distinct task:

   * **Task environment:** Responsible for setting up the initial environment state, calculating the task reward, and checking whether an episode is done.
   * **Robot environment:** Responsible for reading the sensor data and controlling the actuators.
   * **Gazebo environment:** Responsible for connecting the Task and Robot environments to the Gazebo simulation.

Within the :ros-gazebo-gym:`ROS Gazebo Gym <>` framework, you'll find two distinct **gazebo** environments: :class:`~ros_gazebo_gym.robot_gazebo_env.RobotGazeboEnv` and
:class:`~ros_gazebo_gym.robot_gazebo_goal_env.RobotGazeboGoalEnv`. As these Gazebo environments directly inherit from the `gym.Env`_ and `gym.GoalEnv`_ classes and the
**robot** and **task** environments again inherit from these **gazebo** environments, all :ros-gazebo-gym:`ROS Gazebo Gym <>` task environments can be directly imported
like any other conventional gymnasium environment.

.. _`gym.Env`:  https://gymnasium.farama.org/api/env/
.. _`gym.GoalEnv`: https://robotics.farama.org/content/multi-goal_api/#goalenv
.. _`the construct`: https://www.theconstructsim.com/

Contents
========

.. toctree::
   :maxdepth: 3
   :caption: Getting Started

   get_started/install
   get_started/usage
   get_started/envs

.. toctree::
   :maxdepth: 3
   :caption: Development

   dev/contributing.rst
   dev/add_nev_env.rst
   dev/doc_dev.rst
   dev/license.rst
   dev/openai_ros_diff.rst

.. toctree::
   :maxdepth: 2
   :caption: API Documentation

   autoapi/index.rst

.. toctree::
   :maxdepth: 3
   :caption: Etc.

   etc/acknowledgements

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
