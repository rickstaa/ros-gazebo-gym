# Changelog

All notable changes to this project will be documented in this file. See [standard-version](https://github.com/conventional-changelog/standard-version) for commit guidelines.

### [1.1.10](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.1.9...v1.1.10) (2022-02-04)

### [1.1.9](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.1.8...v1.1.9) (2022-02-04)

### [1.1.8](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.1.7...v1.1.8) (2022-02-04)

### [1.1.7](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.1.6...v1.1.7) (2022-02-03)

### [1.1.6](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.1.5...v1.1.6) (2022-02-03)

### [1.1.5](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.1.4...v1.1.5) (2022-02-03)

### [1.1.4](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.1.3...v1.1.4) (2022-02-03)

### [1.1.3](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.1.2...v1.1.3) (2022-02-03)

### [1.1.2](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.1.1...v1.1.2) (2022-02-03)

### [1.1.1](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.1.0...v1.1.1) (2022-02-03)

## 1.1.0 (2022-02-03)


### Features

* add ability to turn of reset simulation log statement ([bacdb75](https://github.com/rickstaa/ros-gazebo-gym/commit/bacdb758533ebea0cadc7c828139f71572ecc626))
* add auto ros master and ros init start feature ([341a84f](https://github.com/rickstaa/ros-gazebo-gym/commit/341a84ff5408323adc5d8a0e006b30b4c14473fb))
* add collision penalty ([0881faa](https://github.com/rickstaa/ros-gazebo-gym/commit/0881faac51ddc1835ffa1efd1e9eac8579fe910b))
* add Panda environment ([b9341c1](https://github.com/rickstaa/ros-gazebo-gym/commit/b9341c1f568618a526aa941bd60785994fd57f7e))
* **controllers_connection:** add auto controllers retrieval ([f148e34](https://github.com/rickstaa/ros-gazebo-gym/commit/f148e345df46041e21162378ce5c53a5a3f6f707))
* **gazebo_connection:** improve 'set_physics_properties' feature ([1d969df](https://github.com/rickstaa/ros-gazebo-gym/commit/1d969dfdd6550c1028751bd885793be9f96b2c77))
* **gazebo_env:** improve controller reset behavoir ([01a92db](https://github.com/rickstaa/ros-gazebo-gym/commit/01a92db7d5c73f3c0afd730fbfffba9297e5a8c8))
* **helpers:** add recursive clone option ([3be4c81](https://github.com/rickstaa/ros-gazebo-gym/commit/3be4c81e4a68c7fd8767fc6d2b80b9efeb5cb163))
* **helpers:** fix environment 'max_episode_steps' bug ([5fc1faa](https://github.com/rickstaa/ros-gazebo-gym/commit/5fc1faa97b4463541e25ae1e716eec932aed8af6))
* improves task environment import/make behaviour ([b579e2b](https://github.com/rickstaa/ros-gazebo-gym/commit/b579e2bc5e8c4c604d5ee3bc7daa0ee735cc62a8))
* **robot_gazebo_Env:** add '_set_init_gazebo_variables' method ([5e980c5](https://github.com/rickstaa/ros-gazebo-gym/commit/5e980c5cedf9012d0879b50088b959e26b1faabf))
* **robot_goal_env:** make 'reset_robot_pose' optional ([b87d196](https://github.com/rickstaa/ros-gazebo-gym/commit/b87d196c58198389131675025a37a2132c030f2a))
* **roslauncher:** add catkin build capability ([2eaf3d8](https://github.com/rickstaa/ros-gazebo-gym/commit/2eaf3d84bfdf7b853117511b04c7c2bc52eeb169))
* **roslauncher:** add catkin re-build to ROSLauncher ([4196e79](https://github.com/rickstaa/ros-gazebo-gym/commit/4196e790604cc83436c1d7befc3d5d4cc19de0e2))
* **roslauncher:** add kwargs to the ROSLauncher launch method ([b484ca3](https://github.com/rickstaa/ros-gazebo-gym/commit/b484ca367929c79264c16a7e20754c448101b98e))
* **start_openai_ros_env:** forward kwargs to env ([c2ae43a](https://github.com/rickstaa/ros-gazebo-gym/commit/c2ae43ac506a6cac386dd3834e21ad8f11cf2f7c))


### Bug Fixes

* **cmakelist:** fix catkin_package dependency syntax ([f96f5f0](https://github.com/rickstaa/ros-gazebo-gym/commit/f96f5f0e6b040e017b3b44d1d030d8134efb6151))
* **controllers_connection:** fix controller stop bug ([a8b7c05](https://github.com/rickstaa/ros-gazebo-gym/commit/a8b7c052580c1fe307817b39d74bfa7e61da5a83))
* **controllers_connection:** Fix gazebo paused switch problem ([6ce9736](https://github.com/rickstaa/ros-gazebo-gym/commit/6ce973652e1a6640140a2e6475203dec37ebe7ab))
* **controllers_connection:** fixes controller manager namespace problem ([fb729f9](https://github.com/rickstaa/ros-gazebo-gym/commit/fb729f96f64568dac84363a36b37b218228c6ec3))
* fix rosdep system dependency install bug ([88e9ea1](https://github.com/rickstaa/ros-gazebo-gym/commit/88e9ea1a637236b64ba304079d2d1f0218d524b6))
* fix several turtlebot environment bugs ([4e342f5](https://github.com/rickstaa/ros-gazebo-gym/commit/4e342f5e53a1911d2f62ac4a05e9800a6c4bd02b))
* fixes controllers switch bug ([4af383e](https://github.com/rickstaa/ros-gazebo-gym/commit/4af383efd6e7db527b1f030683f40469da11679b))
* improves Gazebo reset behavoir ([1bf899f](https://github.com/rickstaa/ros-gazebo-gym/commit/1bf899fa5a72b3c23afeacc8943eb040ca221f8e))
* **roslauncher:** add ROS system dependency install capability ([f9ad537](https://github.com/rickstaa/ros-gazebo-gym/commit/f9ad537aaba8c1c6e30bb2b45fd699a943c495a3))
* **roslauncher:** fix catkin build workspace source bug ([5230ea2](https://github.com/rickstaa/ros-gazebo-gym/commit/5230ea2f0334d8a73eaba5358ea07fb68989baaf))
* **roslauncher:** fix github clone stdout printing bug ([71af3ec](https://github.com/rickstaa/ros-gazebo-gym/commit/71af3ec55d606e59292ad77a32552e72b7f0931e))
* **roslauncher:** fix ROS workspace source bug ([cf9b59d](https://github.com/rickstaa/ros-gazebo-gym/commit/cf9b59d01bff91d09ef57dde702a64e3234c486a))
* **roslauncher:** make sure child processes are terminated ([0517b90](https://github.com/rickstaa/ros-gazebo-gym/commit/0517b90c259136f5bb6b73c6c30dda00adc4a83f))
