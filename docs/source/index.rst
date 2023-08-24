.. ros_gazebo_gym documentation master file, created by
   sphinx-quickstart on Tue Jul 10 17:48:23 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to ros_gazebo_gym's documentation!
==========================================

.. figure:: /images/ros_gazebo_gym.png
   :alt: ROS Gazebo Gym

The ROS Gazebo Gym framework provides all the tools required to create ROS based :gymnasium:`gymnasium <>` robot environments.

RL algorithms have achieved impressive results in games and simulated environments in the last few years. For example, the Deep-mind team trained
an RL algorithm that outperforms humans on all of the `Atari games <https://arxiv.org/abs/2003.13350>`_ and
`another one that even beat professional Dota 2 players <https://arxiv.org/abs/1912.06680>`_. However, much work needs to be done to translate these results
to real-world robots. Due to safety and time constraints, most RL algorithms can not be directly trained on real robots. As a result, people have to rely
on simulations. :gymnasium:`Gymnasium` provides a valuable toolkit for developing and comparing reinforcement learning algorithms. This toolkit,
however, is not directly compatible with the simulated environments often used in robotics research.

The :ros-gazebo-gym:`ros_gazebo_gym <>` framework provides a way to translate ROS Gazebo simulations into gymnasium environments easily. While doing this, the focus lies on
delivering real-world ready solutions, meaning algorithms trained in simulation can readily be applied to the Real robot. We hope to create a common ground for
people who use RL with real robots and accelerate the research in this area.

Package structure
-----------------

The structure of this package was based on the `openai_ros`_ package created by `the construct`_. As a result, each ROS Gazebo gymnasium environment is
divided into three classes: a **task** environment, a **robot** environment and a **gazebo** environment.

.. figure:: /images/ros_gazebo_gym_diagram.png
   :alt: ROS Gazebo Gym Structure Diagram

Using this compartmentalised structure makes the code-base easier to understand and is easier to extend. Each of these classes is responsible for one distinct task:

   * **Task environment:** Responsible for setting up the initial environment state, calculating the task reward, and checking whether an episode is done.
   * **Robot environment:** Responsible for reading the sensor data and controlling the actuators.
   * **Gazebo environment:** Responsible for connecting the Task and Robot environments to the Gazebo simulation.

Since the **Gazebo** environment directly inherits from the `gym.Env`_ class, each :ros-gazebo-gym:`ros_gazebo_gym <>` task environment can be directly imported
like any other gymnasium environment. The :ros-gazebo-gym:`ros_gazebo_gym <>` package covers all the Gazebo and ROS related components. It installs and builds all the dependencies,
makes sure that a ROS master is running, initialises ROS, starts gazebo and spawns the robot.


.. toctree::
   :maxdepth: 3
   :caption: Getting Started

   user/installation
   user/usage

.. toctree::
   :maxdepth: 3
   :caption: Environments

   envs/robot_envs

.. toctree::
   :maxdepth: 3
   :caption: Developer Zone

   dev/api/api.rst
   dev/add_nev_env.rst
   dev/release_dev.rst
   dev/doc_dev.rst
   dev/openai_ros_diff.rst
   dev/license.rst

.. toctree::
   :maxdepth: 3
   :caption: Etc.

   etc/acknowledgements
   etc/author

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

.. _`gym.Env`:  https://gymnasium.farama.org/api/env/
.. _`openai_ros`: (http://wiki.ros.org/openai_ros
.. _`the construct`: https://www.theconstructsim.com/
