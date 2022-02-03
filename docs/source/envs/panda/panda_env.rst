Panda Environment
=================

.. important::

   This environment requires the `libfranka`_ library to be installed to work. You are recommended to build this
   library from source according to the `frankaemika documentation`_.

The Panda robot is a high-performance 7DOF research robot created by `Franka Emika`_. It
is very sensitive, agile and contains torque sensors at each joint, allowing
adjustable stiffness/compliance and advanced torque control.

It is controlled through the `Franka Control Interface`_ (FCI), which can be accessed
via several open-source components provided on `GitHub <https://github.com/frankaemika>`_.
These components contain both a low-level C++ library called `libfranka`_  and a high-level
ROS interface called `franka_ros`_. The high-level interface provides support for both ROS
Control and MoveIt!

.. figure:: ../../images/panda/panda_robot.png
   :alt: Panda Robot

It also comes with a simulated version (i.e. `franka_gazebo`_) that uses all the SDK systems
that the real robot does, so anything you develop for the simulation should work
seamlessly with the real robot.

.. figure:: ../../images/panda/panda_sim.png
   :alt: Panda Reach simulation

.. attention::
   There are some small differences between the real and simulated robot:

      - The masses and inertias are not precisely equal between the simulated and the real robot.
      - The simulation motion generators are not equal to the ones contained in the
        `FCI <https://frankaemika.github.io/docs/libfranka.html#realtime-commands>`_.

.. _franka_gazebo: https://frankaemika.github.io/docs/franka_ros.html#franka-gazebo

Control types
-------------

The :ros_gazebo_gym:`ros_gazebo_gym <>` version of the Panda environment allows for four types of control: ``effort``, ``position``, ``trajectory`` and
``end-effector`` control. End-effector control is implemented through `MoveIt`_.

.. _MoveIt: https://moveit.ros.org/

.. seealso::
   All the components used to create the :ros_gazebo_gym:`ros_gazebo_gym <>` Panda environment are contained in the `panda-gazebo`_ ROS workspace
   package. This package is automatically downloaded and built by :ros_gazebo_gym:`ros_gazebo_gym <>` package when running one of the panda task
   environments.

.. _panda-gazebo: https://github.com/rickstaa/panda-gazebo

Task environments
-----------------

The tasks environments that are in the panda :ros_gazebo_gym:`ros_gazebo_gym <>` package were based on the fetch environments that are found in the `openai gym`_
package. It currently contains the following task environments:

* :class:`PandaReach-v0 <ros_gazebo_gym.task_envs.panda.panda_reach>`: Move Panda to a goal position.
* :class:`PandaPickAndPlace-v0 <ros_gazebo_gym.task_envs.panda.panda_pick_and_place>`: Lift a block into the air.
* :class:`PandaPush-v0 <ros_gazebo_gym.task_envs.panda.panda_push>`: Push a block to a goal position.
* :class:`PandaSlide-v0 <ros_gazebo_gym.task_envs.panda.panda_slide>`: Slide a puck to a goal position.


System dependencies
-------------------

The Panda task environments require the following system dependencies to be installed. These dependencies should have been fetched and install by the :ros_gazebo_gym:`ros_gazebo_gym <>` package
the first time a task environment is imported:

* `ROS noetic <http://wiki.ros.org/noetic>`_.
* The :panda_gazebo:`panda_gazebo <>` package.
* The `libfranka`_ library.

.. _`Franka Emika`: https://www.franka.de
.. _`Franka Control Interface`: https://frankaemika.github.io/docs/
.. _`libfranka`: https://frankaemika.github.io/docs/libfranka.html
.. _`franka_ros`: https://frankaemika.github.io/docs/franka_ros.html
.. _`openai gym`: https://gym.openai.com/envs/#robotics
.. _`frankaemika documentation`: https://frankaemika.github.io/docs/installation_linux.html
