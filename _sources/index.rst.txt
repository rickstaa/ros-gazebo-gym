=========================================
Welcome to ros_gazebo_gym's documentation
=========================================

.. image:: /images/ros_gazebo_gym.png
   :alt: ROS Gazebo Gym

Welcome to the ROS Gazebo Gym framework! This framework provides all the tools required for integrating :ros:`ROS <>` and :gazebo:`Gazebo <>` with :gymnasium:`gymnasium <>`, streamlining the development and training of RL algorithms in realistic robot simulations. It comes equipped with various robot simulation environments, ready for use and further customization.

RL algorithms have recently achieved impressive results in games and simulated environments. For example, the DeepMind team trained an RL algorithm that outperforms humans in all of the `Atari games`_ and `another one that beat professional Dota 2 players`_. However, translating these results to real-world robots requires considerable work. Most RL algorithms cannot be trained directly on real robots due to safety and time constraints, necessitating reliance on simulations. While :gymnasium:`Gymnasium <>` provides a valuable toolkit for developing and comparing reinforcement learning algorithms, it is not directly compatible with the simulated environments often used in robotics research.

The :ros-gazebo-gym:`ros-gazebo-gym <>` framework effectively translates ROS Gazebo simulations into gymnasium environments, optimizing for real-world applicability. This enables the practical application of simulation-trained algorithms to real robots. However, users should be mindful of `the real-to-sim gap`_ and the necessity for additional adjustments and safety evaluations. Our objective with the ros_gazebo_gym package is to provide a foundational tool for RL research with real robots to bridge this gap and foster advancements in the field.

.. _`Atari games`: https://arxiv.org/abs/2003.13350
.. _`another one that beat professional Dota 2 players`: https://arxiv.org/abs/1912.06680
.. _`games and simulations`: https://arxiv.org/abs/1912.06680
.. _`the real-to-sim gap`: https://ieeexplore.ieee.org/abstract/document/9308468/

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
