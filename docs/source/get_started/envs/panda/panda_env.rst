=================
Panda environment
=================

Introduction
============

.. image:: /images/panda/panda_robot.png
   :alt: Panda Robot

The Panda robot, designed by `Franka Emika`_, is a sophisticated 7DOF research robot. Characterized by its sensitivity, agility,
and torque sensors at each joint, the Panda robot offers adjustable stiffness/compliance and advanced torque control. It is
controlled through the `Franka Control Interface`_ (FCI), which can be accessed through open-source  components available on
`GitHub <https://github.com/frankaemika>`_. These components include the `libfranka`_ C++ library for low-level control and
the `franka_ros`_ high-level ROS interface that supports both ROS Control and MoveIt! The Panda robot also has a simulated 
counterpart known as :franka-ros:`franka_gazebo <tree/develop/franka_gazebo>`. This simulation closely mirrors the SDK
systems of the real robot, ensuring that developments in the simulation translate seamlessly to the real robot.

.. image:: /images/panda/panda_sim.png
   :alt: Panda Reach simulation

.. attention::
   It's important to note that there are minor disparities between the real and simulated Panda robot:

   - Masses and inertias differ between the simulated and real robot.
   - Simulation motion generators do not match those in the `FCI`_.

.. _`Franka Emika documentation`: https://frankaemika.github.io/docs/installation_linux.html
.. _`Franka Emika`: https://www.franka.de
.. _`Franka Control Interface`: https://frankaemika.github.io/docs/
.. _`franka_ros`: https://frankaemika.github.io/docs/franka_ros.html
.. _`FCI`: https://frankaemika.github.io/docs/libfranka.html#realtime-commands

System dependencies
===================

The Panda environment requires the following system dependencies to be installed on the host machine:

- :ROS:`ROS Noetic <noetic>`.
- The `libfranka`_ library **(automatically installed by rosdep)**.

.. _`libfranka`: https://frankaemika.github.io/docs/libfranka.html

Control types
=============

In the :ros-gazebo-gym:`ros_gazebo_gym <>` version of the Panda environment, four control types are supported: ``effort``, ``position``, ``trajectory``, and
``end-effector`` control. The end-effector control is facilitated through `MoveIt`_.

.. _MoveIt: https://moveit.ros.org/

Task environments
=================

The panda :ros-gazebo-gym:`ros_gazebo_gym <>` package comprises task environments adapted from the fetch environments in the :gymnasium:`gymnasium <>` package. Currently, it includes the following task environments:

* :class:`PandaReach-v1 <ros_gazebo_gym.task_envs.panda.panda_reach>`: Move the Panda robot to a goal position.
* :class:`PandaPickAndPlace-v1 <ros_gazebo_gym.task_envs.panda.panda_pick_and_place>`: Lift and place a block at a goal.
* :class:`PandaPush-v1 <ros_gazebo_gym.task_envs.panda.panda_push>`: Push a block to a goal position.
* :class:`PandaSlide-v1 <ros_gazebo_gym.task_envs.panda.panda_slide>`: Slide a puck to a goal position.

.. note::
   The components responsible for creating the :ros-gazebo-gym:`ros_gazebo_gym <>` Panda environment are enclosed within the :panda-gazebo:`panda-gazebo <>` ROS workspace package. This package is automatically downloaded
   and built by the :ros-gazebo-gym:`ros_gazebo_gym <>` package when running one of the panda task environments.

Known Issues
============

This document outlines the known issues you may encounter when using the Panda environment. These issues are caused by upstream bugs in the `franka_ros`_ package, which the :panda-gazebo:`panda_gazebo <>` package
uses to create the simulated Panda robot. We recommend checking :panda-gazebo:`the issues page <issues>` on the :panda-gazebo:`panda-gazebo GitHub repository <>` for updates.

Gravity Compensation Bug
-------------------------

- **Issue**: Gravity compensation is not working properly when the robot is effort-controlled.
- **Reference**: `panda-gazebo#39 <https://github.com/rickstaa/panda-gazebo/issues/39>`_

Due to an upstream bug in the :panda-gazebo:`franka_gazebo <>` package, the gravity compensation feature may not function as expected. 
This issue can be mitigated by switching the physics engine from `ODE`_ to `DART`_ using the ``physics`` argument in the ``simulation.launch`` file:

.. code-block:: bash

    roslaunch panda_gazebo simulation.launch physics:=Dart

Simulation Crashes with DART Physics
--------------------------------------

- **Issue**: The simulation sometimes crashes when the `DART`_ physics engine is used and the gripper is controlled.
- **Reference**: `panda-gazebo#196 <https://github.com/rickstaa/panda-gazebo/issues/196>`_

Users may experience occasional simulation crashes when using the `DART`_ physics engine and controlling the gripper. We
recommend switching to the `ODE`_ physics engine when using the gripper.

Gripper Problems when Vertical
------------------------------

- **Issue**: The gripper is not working properly when being vertical to the ground.
- **Reference**: `panda-gazebo#33 <https://github.com/rickstaa/panda-gazebo/issues/33>`_

Due to incorrectly tuned PID gains in the :panda-gazebo:`franka_gazebo <>` package, the gripper may not function properly when it is oriented
vertically to the ground.

.. _ODE: http://www.ode.org/
.. _DART: https://dartsim.github.io/
