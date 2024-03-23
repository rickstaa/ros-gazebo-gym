# Custom ROS-gazebo-gym rosdep rules

This folder contains custom [rosdep](https://wiki.ros.org/rosdep) rules for the `ros-gazebo-gym` package. These rules were created to resolve conflicting requirements between the `python3-gymnasium-pip` and `python3-gymnasium-robotics` rules and the `python3-numpy` rule on [Ubuntu 20.04](https://releases.ubuntu.com/focal/). Specifically, `gymnasium` requires `numpy>=1.20.1`, but the `python3-numpy` rule installs `numpy==1.17.4` (see https://github.com/ros/rosdistro/issues/38332).

> \[!IMPORTANT]\
> While the steps provided here will work, it is recommended that you use a virtual environment to keep your ROS system packages separate from your project-specific packages. This can help avoid conflicts and ensure reproducibility. To create a virtual environment, you can use the [venv](https://docs.python.org/3/library/venv.html) package and install the correct Numpy, gymnasium and gymnasium-robotics versions directly using [pip](https://pypi.org/project/pip/). When creating the virtual environment, include the `--system-site-packages` flag so that the ROS system packages are available in the virtual environment.

## Usage

To use these rules, follow these steps:

1.  Copy or download the [19-ros-gazebo-gym.list](https://github.com/rickstaa/ros-gazebo-gym/tree/noetic/rosdep/19-ros-gazebo-gym.list) file to the `/etc/ros/rosdep/sources.list.d/` folder.
2.  Run `rosdep update` to update the rosdep database.
3.  Run `rosdep install --reinstall --from-path src --ignore-src -r -y` to install the ROS dependencies of the `ros_gazebo_gym` package. Note that the `--reinstall` flag is necessary because `python3-numpy` is already installed when ROS is installed.

## Restoring Default Rules

If you want to restore the default rules, follow these steps:

1.  Remove the [19-ros-gazebo-gym.list](https://github.com/rickstaa/ros-gazebo-gym/tree/noetic/rosdep/19-ros-gazebo-gym.list) file from the `/etc/ros/rosdep/sources.list.d/` folder.
2.  Run `rosdep update` to update the rosdep database.
3.  Run `rosdep install --reinstall ros-gazebo-gym` to install the dependencies of the `ros-gazebo-gym` package. Note that the `--reinstall` flag is necessary because `python3-numpy` is already installed when ROS is installed.
