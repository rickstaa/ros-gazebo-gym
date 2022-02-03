============
Installation
============

Package installation
~~~~~~~~~~~~~~~~~~~~

To use the :ros_gazebo_gym:`ros_gazebo_gym <>` task environment, you first have to install `ROS noetic`_ (see the `ROS documentation`_).
After you install ROS, you can then `create a catkin workspace <http://wiki.ros.org/catkin/Tutorials/create_a_workspace>`_
and install the :ros_gazebo_gym:`ros_gazebo_gym <>` system dependencies using the following command:

.. code-block:: bash

    rosdep install --from-path src --ignore-src -r -y

After the system dependencies are installed, you can build the :ros_gazebo_gym:`ros_gazebo_gym <>` package using the following command:

.. code-block: bash

    'catkin build -DCMAKE_BUILD_TYPE=Debug

.. warning::

    When using ROS inside an anaconda environment, you might run into problems (see `this issue`_). If you want to use the :ros_gazebo_gym:`ros_gazebo_gym <>` framework in a virtual environment, you are advised to use the default python virtual environment package instead. A virtual env can be created using
    this package with the following command:

        .. code:: bash

            python -m venv ./blc --system-site-packages

    You can then source this environment using the ``. ./blc/bin/activate`` command. The  ``--system-site-packages`` flag makes sure that
    the virtual environment has access to the system site-packages. Alternatively, you can also use the
    `RoboStack ros-noetic <https://github.com/RoboStack/ros-noetic>`_ `conda-forge <https://conda-forge.org/>`_ packages
    (see this `blog post <https://medium.com/robostack/cross-platform-conda-packages-for-ros-fa1974fd1de3>`_ for more
    information.

Environment dependencies
~~~~~~~~~~~~~~~~~~~~~~~~

The :ros_gazebo_gym:`ros_gazebo_gym <>` package tries to fetch and install the environment dependencies the first time a task environment is
imported. It could, however, happen that not all the environment dependencies were installed successfully. If this happens, you are advised to run the aforementioned
``rosdep`` command and re-build the catkin workspace. A list of all the required environment dependencies can be found inside the documentation of
each task environment (see :ref:`robot_envs`).

.. _`this issue`: https://answers.ros.org/question/256886/conflict-anaconda-vs-ros-catking_pkg-not-found/
.. _`ROS noetic`: http://wiki.ros.org/noetic
.. _`ROS documentation`: http://wiki.ros.org/noetic
