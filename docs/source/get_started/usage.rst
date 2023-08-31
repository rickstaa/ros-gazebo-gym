==========
How to use
==========

This section provides instructions on how to use the ROS Gazebo Gym framework to train reinforcement learning algorithms.

General usage instructions
==========================

The :ros-gazebo-gym:`ROS Gazebo Gym <>` framework contains several gymnasium robotics task environments that can be used to train RL algorithms. Please see 
the :ref:`environments <envs>` section for more information about all available task environments. All the environments in the :ros-gazebo-gym:`ROS Gazebo Gym <>` 
framework can be imported like any other :gymnasium:`gymnasium <>` environment. The environments will be registered under the ``gymnasium`` namespace when imported 
from the ``ros_gazebo_gym`` package or any of its submodules. You can also prefix the environment name with the ``ros_gazebo_gym`` module name prefix, separated by
a colon, to use it directly inside the `gym.make` function (e.g. ``ros_gazeb_gym:PandaReach-v1``).

.. important::

   Since the task environments found in the ROS Gazebo Gym are created in Gazebo and ROS, you have to make sure that you first build and source the ``ros_gazebo_gym`` catkin workspace
   before importing any task environments.

Usage Examples
==============

A collection of usage examples showcasing how to utilize the ROS Gazebo Gym framework can be discovered in the :ros-gazebo-gym-examples:`ros_gazebo_gym_examples <>` package. These illustrative examples are
tailored for integration with the `stable-baselines3`_ package. The current selection of examples includes:

- :ros-gazebo-gym-examples:`launch/start_panda_training.launch <>`: This launch file initiates a training session for the ``PandaReach-v1`` task environment using the `Stable Baselines SAC algorithm`_.

To embark on a training session for a reaching task involving a (simulated) `Panda Emika Franka`_ robot using the `Stable Baselines SAC algorithm`_, follow these steps:

1. Clone the `ros_gazebo_gym_examples` package within the Catkin workspace established for the ``ros_gazebo_gym`` package.
2. Complete the installation of all requisite ROS dependencies and perform the catkin workspace reconstruction (refer to :ref:`install_dependencies` and :ref:`build_package`).
3. Source the Catkin workspace (e.g., execute ``. ../develop/setup.bash``).
4. Launch any of the examples from the :ros-gazebo-gym-examples:`ros_gazebo_gym_examples <>` package using the ``roslaunch`` command.

Here's a demonstration of how to launch the training example:

.. code-block:: bash

   roslaunch ros_gazebo_gym_examples start_training.launch

.. note::

   To streamline the installation process, the :ros-gazebo-gym-ws:`ros-gazebo-gym-ws <>` repository is provided. This repository encompasses both the :ros-gazebo-gym:`ros_gazebo_gym <>` and :ros-gazebo-gym-examples:`ros_gazebo_gym_examples <>` package repositories. You can effortlessly incorporate these repositories into your Catkin workspace using the subsequent command:

   .. code-block:: bash

      git clone --recurse-submodules https://github.com/rickstaa/ros-gazebo-gym-ws.git src

   The usage of the ``--recurse-submodules`` argument ensures the cloning of all submodules. If you have already cloned the repository and neglected the `--recurse-submodule` argument, you can rectify this by employing the following git command:

   .. code-block:: bash

      git submodule update --init --recursive

.. _`stable-baselines3`: https://stable-baselines3.readthedocs.io
.. _`Stable Baselines SAC algorithm`: https://stable-baselines3.readthedocs.io/en/master/modules/sac.html
.. _`Panda Emika Franka`: https://www.franka.de/

Task environment configuration
==============================

Configuring the main parameters of a :ros-gazebo-gym:`ROS Gazebo Gym <>` task environment is a straightforward process using the :func:`gym.make` function. For instance, to modify settings like the maximum number of steps per episode, 
the reward mechanism, and the control type employed in the ``PandaReach-v1`` task environment, follow this example:

.. code-block:: python

   import gym

   env = gym.make("ros_gazebo_gym:PandaReach-v1", max_episode_steps=1000, positive_reward=False, control_type="effort")

Additionally, each environment provides a corresponding ``yaml`` configuration file. These files are invaluable for precise fine-tuning and reside in the ``config`` folder alongside the respective task environment. When you create
a task environment, these configuration files are automatically loaded. The parameters in these configuration files are also directly accessible through the ROS parameter server. 

.. _troubleshooting:

Troubleshooting
===============

This section provides solutions to common issues when using the ROS Gazebo Gym package.

.. _virtual_environment:

Virtual environments and ROS
----------------------------

When incorporating the :ros-gazebo-gym:`ROS Gazebo Gym <>` framework within a virtual environment to maintain the integrity of your system's Python installation, you can utilize the `venv`_ package. Execute the
following command to establish a virtual environment within the ``ros-gazebo-gym`` folder:

.. code-block:: bash

   python -m venv ./ros-gazebo-gym --system-site-packages

It's crucial to include the ``--system-site-packages`` flag. This inclusion ensures that the virtual environment gains access to the ROS system packages. Subsequently, you can activate this environment using
the command: ``. ./ros-gazebo-gym/bin/activate``.

.. attention::

   You might encounter complications when working with ROS within an `anaconda`_ environment (refer to `this issue`_). If you intend to utilize the :ros-gazebo-gym:`ROS Gazebo Gym <>` framework within a
   virtual environment, it's recommended to opt for the default Python `virtual environment package <https://docs.python.org/3/library/venv.html>`_. Alternatively, consider leveraging the `RoboStack ros-noetic`_ packages available on
   `conda-forge`_. For additional insights, consult this enlightening `blog post`_.

.. _venv: https://docs.python.org/3/library/venv.html
.. _`anaconda`: https://www.anaconda.com/
.. _`this issue`: https://github.com/ros/rosdistro/issues/38332
.. _`RoboStack ros-noetic`: https://github.com/RoboStack/ros-noetic
.. _`conda-forge`: https://conda-forge.org/
.. _`blog post`: https://medium.com/robostack/cross-platform-conda-packages-for-ros-fa1974fd1de3

Ubuntu 20.04 compatibility issues
---------------------------------

When deploying the :ros-gazebo-gym:`ros_gazebo_gym <>` package on `Ubuntu 20.04`_, it's possible to run into complications arising from conflicting versions of the :gymnasium:`gymnasium <>` and `Numpy`_
packages. This conflict is detailed in `this issue`_. Should you encounter this situation, a recommended strategy involves utilizing a :ref:`virtual environment <virtual_environment>` and manually
installing the required Python dependencies from the ``requirements.txt`` file using `pip`_.

Execute the following command to install the dependencies:

.. code-block:: bash

   pip install -r requirements.txt

.. note::

   While it's not the preferred approach, we also offer custom rosdep rules to ensure proper installation of the required versions of the :gymnasium:`gymnasium <>` and `Numpy`_ packages, even without utilizing a :ref:`virtual environment <virtual_environment>`. For
   For a deeper dive into this option, please consult the :ros-gazebo-gym:`README.md <tree/noetic/rosdep>` file in the ``rosdep`` folder.

.. _`Ubuntu 20.04`: https://ubuntu.com/download/desktop
.. _`Numpy`: https://numpy.org/
.. _`pip`: https://pip.pypa.io/en/stable/
