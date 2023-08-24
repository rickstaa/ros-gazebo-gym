==========
How to use
==========

General usage instructions
--------------------------

All of the :ros-gazebo-gym:`ros_gazebo_gym <>` environments can be imported like any other :gymnasium:`gymnasium` environment (see the `gym documentation`_). The environments will be
registered under the :gymnasium:`gymnasium` namespace when the ``ros_gazebo_gym`` package or any of its submodules are imported.
Since ROS is used, please be aware that you have to make sure that you first build and source the :ros-gazebo-gym:`ros_gazebo_gym <>` catkin workspace (i.e. ``./develop/setup.bash``).

Usage example
-------------

Several examples for training RL algorithms on the :ros-gazebo-gym:`ros_gazebo_gym <>` task environments are found in the `ros_gazebo_gym_examples`_ package. To test out these examples
first clone the `ros-gazebo-gym-ws`_ package inside a catkin workspace folder. Since the repository contains several git submodules to use all the features, it needs to be cloned using the
``--recurse-submodules`` argument:

.. code-block:: bash

    git clone --recurse-submodules https://github.com/rickstaa/ros-gazebo-gym-ws.git

If you already cloned the repository and forgot the `--recurse-submodule` argument you
can pull the submodules using the following git command:

.. code-block:: bash

    git submodule update --init --recursive

After you cloned the repository, you have to install the system dependencies using the ``rosdep install --from-path src --ignore-src -r -y`` command. After these dependencies are installed,
you can build the ROS packages inside the catkin workspace using the following build command:

.. code-block:: bash

    catkin build

After the catkin workspace is build, you can source the catkin workspace ``. ../develop/setup.bash`` and launched any of the examples found in the `ros_gazebo_gym_examples`_ package using the ``roslaunch`` command. The example
below uses the `SAC algorithm of the stable-baselines`_ package to train a reaching task on a (simulated) `Panda Emika Franka`_ robot.

.. code-block:: bash

    roslaunch ros_gazebo_gym_examples start_training.launch

Task environment configuration
------------------------------

The task environments found in the :ros-gazebo-gym:`ros_gazebo_gym <>` can be configured using ``yaml`` configuration files. These files are inside the ``config`` folder next
to the task environment. The task environment loads the parameters inside these configuration files, making them available through the ROS parameter server.

.. _`gym documentation`: https://gymnasium.farama.org
.. _`ros-gazebo-gym-ws`: https://github.com/rickstaa/ros-gazebo-gym-ws
.. _`ros_gazebo_gym_examples`: https://github.com/rickstaa/ros-gazebo-gym-examples
.. _`SAC algorithm of the stable-baselines`: (https://stable-baselines3.readthedocs.io/en/master/modules/sac.html)
.. _`Panda Emika Franka`: https://www.franka.de/
