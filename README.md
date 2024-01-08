# ROS Gazebo Gym

[![ROS Gazebo Gym](https://github.com/rickstaa/ros-gazebo-gym/actions/workflows/ros_gazebo_gym.yml/badge.svg)](https://github.com/rickstaa/ros-gazebo-gym/actions/workflows/ros_gazebo_gym.yml)
[![ROS Test](https://github.com/rickstaa/ros-gazebo-gym/actions/workflows/ros_test.yml/badge.svg?branch=noetic)](https://github.com/rickstaa/ros-gazebo-gym/actions/workflows/ros_test.yml)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/rickstaa/ros-gazebo-gym)](https://github.com/rickstaa/ros-gazebo-gym/releases)
[![Python 3](https://img.shields.io/badge/Python-3.8%20%7C%203.7%20%7C%203.6-brightgreen)](https://www.python.org/)
[![ROS version](https://img.shields.io/badge/ROS%20versions-Noetic-brightgreen)](https://wiki.ros.org)
[![Contributions](https://img.shields.io/badge/contributions-welcome-brightgreen.svg)](CONTRIBUTING.md)
[![DOI](https://zenodo.org/badge/453634930.svg)](https://zenodo.org/badge/latestdoi/453634930)

The ROS Gazebo Gym framework integrates [ROS](https://www.ros.org/) and [Gazebo](http://gazebosim.org/) with [gymnasium](https://gymnasium.farama.org/) to facilitate the development and training of RL algorithms in realistic robot simulations. It comes equipped with several ready-to-use simulation environments, allowing for a diverse range of applications and experimentation. This framework provides a streamlined way to apply simulation-trained algorithms to actual robots, thereby enhancing their real-world applicability. Our goal with ROS Gazebo Gym is to establish a robust foundation for RL research in robotics, aiding in the advancement of practical, real-world robot control."

## Installation and Usage

Please see the [docs](https://rickstaa.github.io/ros-gazebo-gym) for installation and usage instructions.

## Contributing

We use [husky](https://github.com/typicode/husky) pre-commit hooks and github actions to enforce high code quality. Please check the [CONTRIBUTING.md](https://github.com/rickstaa/ros-gazebo-gym/blob/noetic/CONTRIBUTING.md) before contributing to this repository.

> \[!NOTE]\
> We used [husky](https://github.com/typicode/husky) instead of [pre-commit](https://pre-commit.com/), which is more commonly used with Python projects. This was done because only some tools we wanted to use were possible to integrate the Please feel free to open a [PR](https://github.com/rickstaa/ros-gazebo-gym/pulls) if you want to switch to pre-commit if this is no longer the case.
