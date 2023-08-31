============
Installation
============

This guide provides step-by-step instructions on installing the :ros-gazebo-gym:`ROS Gazebo Gym package <>` and its dependencies. Follow these steps to get started:

1. Install system dependencies:

   The following system dependencies are required to run the ROS Gazebo Gym package:

   * :ROS:`ROS Noetic - Desktop full <noetic>`.
   * `Python 3`_.

   After these dependencies are installed, you can `create a catkin workspace`_ and clone the ROS Gazebo Gym package inside the ``src`` folder of the workspace.

2. Build the package:

   After installing the ROS Gazebo Gym package and its dependencies, you can use the `catkin_make`_ or `catkin-tools`_ packages to build it.

3. Install the ROS Gazebo Gym gymnasium environment dependencies.

Below, each of these steps is explained in more detail.

.. _`Python 3`: https://www.python.org/downloads/
.. _`create a catkin workspace`: httpss://wiki.ros.org/catkin/Tutorials/create_a_workspace
.. _`catkin_make`: https://wiki.ros.org/catkin/commands/catkin_make
.. _`catkin-tools`: https://catkin-tools.readthedocs.io/en/latest/


.. _install_dependencies:

Install system dependencies
===========================

To install the system dependencies, follow these steps:

1. Install :ROS:`ROS Noetic - Desktop full <noetic>` and `Python 3`_.
2. Create a catkin workspace:

.. code-block:: bash

   mkdir -p ~/ros_gazebo_gym_ws/src
   cd ~/ros_gazebo_gym_ws/src

3. Clone the :ros-gazebo-gym:`ROS Gazebo Gym package <>` inside the ``src`` folder of the workspace:

.. code-block:: bash

   git clone https://github.com/rickstaa/ros-gazebo-gym.git


4. Install the :ros-gazebo-gym:`ROS Gazebo Gym package <>` dependencies using the `rosdep`_ package:

.. code-block:: bash

   rosdep install --from-path src --ignore-src -r -y

.. _`rosdep`: http://wiki.ros.org/rosdep

.. _build_package:

Build the package
=================

You can use the `catkin_make`_ or `catkin-tools`_ packages to build the ROS Gazebo Gym package. To build the package, execute the following commands inside the root of the catkin
workspace:

.. code-block:: bash

   catkin_make

or

.. code-block:: bash

   catkin build

Install environment dependencies
================================

Each of the :ref:`robotics gymnasium environments <envs>` in the :ros-gazebo-gym:`ros_gazebo_gym <>` package has its own dependencies. The :ros-gazebo-gym:`ros_gazebo_gym <>` package
tries to install these dependencies automatically when the environment is imported for the first time. However, not all the dependencies might be installed. If this happens, you
should run the aforementioned ``rosdep`` command and re-build the catkin workspace. A list of all the required environment dependencies can be found inside the documentation of
each :ros-gazebo-gym:`ROS Gazebo Gym <>` robotics gymnasium environment (see :ref:`envs`).

.. _`this issue`: https://answers.ros.org/question/256886/conflict-anaconda-vs-ros-catking_pkg-not-found/
