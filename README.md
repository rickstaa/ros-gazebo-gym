# ROS Gazebo Gym

[![ROS Gazebo Gym](https://github.com/rickstaa/ros-gazebo-gym/actions/workflows/ros_gazebo_gym.yml/badge.svg)](https://github.com/rickstaa/ros-gazebo-gym/actions/workflows/ros_gazebo_gym.yml)
[![ROS Test](https://github.com/rickstaa/ros-gazebo-gym/actions/workflows/ros_test.yml/badge.svg)](https://github.com/rickstaa/ros-gazebo-gym/actions/workflows/ros_test.yml)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/rickstaa/ros-gazebo-gym)](https://github.com/rickstaa/ros-gazebo-gym/releases)
[![Python 3](https://img.shields.io/badge/Python-3.8%20%7C%203.7%20%7C%203.6-brightgreen)](https://www.python.org/)
[![ROS version](https://img.shields.io/badge/ROS%20versions-Noetic-brightgreen)](https://wiki.ros.org)
[![Contributions](https://img.shields.io/badge/contributions-welcome-brightgreen.svg)](CONTRIBUTING.md)
[![DOI](https://zenodo.org/badge/453634930.svg)](https://zenodo.org/badge/latestdoi/453634930)

The ROS Gazebo Gym framework provides all the tools required to create ROS based [gymnasium](https://gymnasium.farama.org/) robot environments.

RL algorithms have achieved impressive results in games and simulated environments in the last few years. For example, the Deep-mind team was able to train an RL algorithm that outperforms humans on all of the [Atari games](https://arxiv.org/abs/2003.13350) and [another one that even beat professional dota 2 players](https://arxiv.org/abs/1912.06680). However, much work needs to be done to translate these results to real-world robots. Due to safety and time constraints, most RL algorithms can not be directly trained on real robots. As a result, people have to rely on simulations. [Gymnasium](https://gymnasium.farama.org/) provides a valuable toolkit for developing and comparing reinforcement learning algorithms. This toolkit, however, is not directly compatible with the simulated environments often used in robotics research.

The [ros-gazebo-gym](https://github.com/rickstaa/ros-gazebo-gym) framework provides a way to translate ROS Gazebo simulations into gymnasium environments easily. While doing this, the focus lies on delivering real-world ready solutions, meaning algorithms trained in simulation can readily be applied to the Real robot. We hope to create a common ground for people who use RL with real robots and accelerate the research in this area.

## Installation and Usage

Please see the [docs](https://rickstaa.github.io/panda-gazebo-gym) for installation and usage instructions.

## Contributing

We use [husky](https://github.com/typicode/husky) pre-commit hooks and github actions to enforce high code quality. Please check the [CONTRIBUTING.md](https://github.com/rickstaa/ros-gazebo-gym/blob/noetic/CONTRIBUTING.md) before contributing to this repository.

> \[!NOTE]\
> We used [husky](https://github.com/typicode/husky) instead of [pre-commit](https://pre-commit.com/), which is more commonly used with Python projects. This was done because only some tools we wanted to use were possible to integrate the Please feel free to open a [PR](https://github.com/rickstaa/ros-gazebo-gym/pulls) if you want to switch to pre-commit if this is no longer the case.
