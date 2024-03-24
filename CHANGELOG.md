# Changelog

All notable changes to this project will be documented in this file. See [standard-version](https://github.com/conventional-changelog/standard-version) for commit guidelines.

## [2.0.1](https://github.com/rickstaa/ros-gazebo-gym/compare/v2.0.0...v2.0.1) (2024-03-24)


### Bug Fixes

* **panda:** fix 'ee_pose_target' not defined bug ([54693d7](https://github.com/rickstaa/ros-gazebo-gym/commit/54693d777261d747d0133a933cd8563012f2cd6d))

## [2.0.0](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.9.5...v2.0.0) (2024-03-24)


### ⚠ BREAKING CHANGES

* block_gripper now is named lock_gripper

### Features

* add ability to pause simulation after step ([#76](https://github.com/rickstaa/ros-gazebo-gym/issues/76)) ([2c7a34a](https://github.com/rickstaa/ros-gazebo-gym/commit/2c7a34ab70347f6e310962e9049215a9b061610e))
* add joint lock feature and rename 'block_gripper' ([#75](https://github.com/rickstaa/ros-gazebo-gym/issues/75)) ([d8e7b90](https://github.com/rickstaa/ros-gazebo-gym/commit/d8e7b901e6df4c6d12b8939c9d7ff87554345353))
* change default physics engine to ODE ([d320d6c](https://github.com/rickstaa/ros-gazebo-gym/commit/d320d6c99734594ad47539ea5abcc03f197c885c))
* **gazebo:** add controller pause/unpause ([#68](https://github.com/rickstaa/ros-gazebo-gym/issues/68)) ([df877f1](https://github.com/rickstaa/ros-gazebo-gym/commit/df877f1a06c41c4a94dc3d99f79847f1e11bdcf8))


### Bug Fixes

* ensure control is compatible with panda_gazebo v2.16.1 ([#70](https://github.com/rickstaa/ros-gazebo-gym/issues/70)) ([6b0f9e0](https://github.com/rickstaa/ros-gazebo-gym/commit/6b0f9e0c566b876ec54ce67fcbaff2d24f0c3715))
* ensure the max vel and acc scaling are correctly applied ([#72](https://github.com/rickstaa/ros-gazebo-gym/issues/72)) ([5c9b5f7](https://github.com/rickstaa/ros-gazebo-gym/commit/5c9b5f7365384f1bd437a2c16f662f53a97c5401))
* fix flake8 errors ([b3f52a7](https://github.com/rickstaa/ros-gazebo-gym/commit/b3f52a729c9e43f85c64b4a6d31b8c959faf4117))
* fix max vel/acc scaling undefined error ([#77](https://github.com/rickstaa/ros-gazebo-gym/issues/77)) ([75a1308](https://github.com/rickstaa/ros-gazebo-gym/commit/75a1308dad44a16020ceccbbc4d4e108c7a1e8b5))
* fix ROS param none value bug ([#74](https://github.com/rickstaa/ros-gazebo-gym/issues/74)) ([b815722](https://github.com/rickstaa/ros-gazebo-gym/commit/b815722d7886aaafa7c70fa9c5f6f2a75a855390))
* **panda:** fix panda joint locking and EE pose retrieval ([#96](https://github.com/rickstaa/ros-gazebo-gym/issues/96)) ([8f4d3b1](https://github.com/rickstaa/ros-gazebo-gym/commit/8f4d3b1aaed0ca5446792dd70649467e56f2c843))
* prevent ros shutdown racing condition ([#71](https://github.com/rickstaa/ros-gazebo-gym/issues/71)) ([d688946](https://github.com/rickstaa/ros-gazebo-gym/commit/d688946d8717352ca6e49c253d8c4642c9f48a83))
* resolve ruamel safe_load deprecation issue ([5eea308](https://github.com/rickstaa/ros-gazebo-gym/commit/5eea30868420e55aa138c4b7fa3050076bf921b0))
* temporary remove ee_frame_offset parameter ([#73](https://github.com/rickstaa/ros-gazebo-gym/issues/73)) ([4e9ce6c](https://github.com/rickstaa/ros-gazebo-gym/commit/4e9ce6c7af197afc88aa4eb602696b9668716241))


### Documentation

* add panda env known issues section ([75c7e67](https://github.com/rickstaa/ros-gazebo-gym/commit/75c7e67e75c08d35b363c7489554303dc2f95ad2))
* fix broken README link ([052e468](https://github.com/rickstaa/ros-gazebo-gym/commit/052e468a5a409f4d65e93e20f54d8291e49b4f77))
* fix doc run commands ([4acf477](https://github.com/rickstaa/ros-gazebo-gym/commit/4acf47768e0c2489d6ddbf449862211b8c4bf3bb))
* improve docs ([0b3ee4e](https://github.com/rickstaa/ros-gazebo-gym/commit/0b3ee4e1e08bb3bf710dfa162f7718496f488a78))
* improve documentation ([a108ff4](https://github.com/rickstaa/ros-gazebo-gym/commit/a108ff43a0b5bedcbd24b138247dec0173140902))
* improve documentation ([e19e85f](https://github.com/rickstaa/ros-gazebo-gym/commit/e19e85f091e9309cf90f1e87807ac1dc9ac84a19))
* improve Panda 'Known Issues' section doc links ([595f708](https://github.com/rickstaa/ros-gazebo-gym/commit/595f7086ef2f535d8a5664f17ca981b650de822e))
* **panda:** improve task env description ([#86](https://github.com/rickstaa/ros-gazebo-gym/issues/86)) ([ebcfd13](https://github.com/rickstaa/ros-gazebo-gym/commit/ebcfd13e08adc63a7ccb7db31a2d106a9e3c600f))

## [1.9.5](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.9.4...v1.9.5) (2023-10-05)


### Documentation

* update libfranka install instructions ([2b8d50c](https://github.com/rickstaa/ros-gazebo-gym/commit/2b8d50ca2745ed01473e4ab73833e873822f2155))

## [1.9.4](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.9.3...v1.9.4) (2023-08-31)


### Documentation

* change ROS test badge branch ([fba4efb](https://github.com/rickstaa/ros-gazebo-gym/commit/fba4efb3f69159f4a0d8c48594363b32e4a64a07))

## [1.9.3](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.9.2...v1.9.3) (2023-08-31)


### Documentation

* fix zenodo config ([#35](https://github.com/rickstaa/ros-gazebo-gym/issues/35)) ([59c8f63](https://github.com/rickstaa/ros-gazebo-gym/commit/59c8f637152e0e1326b02a1b2e8abe2cb800eb5a))
* improve rosdep docs ([#37](https://github.com/rickstaa/ros-gazebo-gym/issues/37)) ([42fb036](https://github.com/rickstaa/ros-gazebo-gym/commit/42fb0360bcbf29442c6edcae23bc26e49cf83921))

## [1.9.2](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.9.1...v1.9.2) (2023-08-31)


### Documentation

* improve usage examples ([#33](https://github.com/rickstaa/ros-gazebo-gym/issues/33)) ([3821bc4](https://github.com/rickstaa/ros-gazebo-gym/commit/3821bc45a6b2543c724bfecbdbd8521d14a5ece9))

## [1.9.1](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.9.0...v1.9.1) (2023-08-31)


### Documentation

* update badges ([#30](https://github.com/rickstaa/ros-gazebo-gym/issues/30)) ([ad24ffd](https://github.com/rickstaa/ros-gazebo-gym/commit/ad24ffd5446505b447ae97cbed61c8fdc1a7a0bb))

## [1.9.0](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.8.0...v1.9.0) (2023-08-31)


### Features

* add step info ([#27](https://github.com/rickstaa/ros-gazebo-gym/issues/27)) ([65267a4](https://github.com/rickstaa/ros-gazebo-gym/commit/65267a4ca164708b93f13429f5f87e6a0dd9c1d7))


### Bug Fixes

* fix panda reach import error ([#22](https://github.com/rickstaa/ros-gazebo-gym/issues/22)) ([0897138](https://github.com/rickstaa/ros-gazebo-gym/commit/08971380ab66c72a891d935b96aa84fcb3dc4844))


### Documentation

* improve documentation ([#26](https://github.com/rickstaa/ros-gazebo-gym/issues/26)) ([a04355d](https://github.com/rickstaa/ros-gazebo-gym/commit/a04355d071f605056d2e015a7cd00035b4a531a6))

## [1.8.0](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.7.0...v1.8.0) (2023-08-28)


### Features

* add rosdeps update warning ([#15](https://github.com/rickstaa/ros-gazebo-gym/issues/15)) ([52b77af](https://github.com/rickstaa/ros-gazebo-gym/commit/52b77afbc760dc8c3f09161f6700276a7dddc120))
* improve ros shutdown and codebase ([#20](https://github.com/rickstaa/ros-gazebo-gym/issues/20)) ([a263b6f](https://github.com/rickstaa/ros-gazebo-gym/commit/a263b6f0208dd24ee943ba27bfbcd68867468f64))
* migrate to gymnasium and cleanup codebase ([#9](https://github.com/rickstaa/ros-gazebo-gym/issues/9)) ([e4fe84d](https://github.com/rickstaa/ros-gazebo-gym/commit/e4fe84d99ac7fad7c849c9a52f054cc8584fda83))


### Bug Fixes

* fix 'PickAndPlace' reset velocities ([#19](https://github.com/rickstaa/ros-gazebo-gym/issues/19)) ([86a1da4](https://github.com/rickstaa/ros-gazebo-gym/commit/86a1da4859973e8c7b3dfc8b338f89d25f70cfdd))
* fix action/observation space dtype ([#17](https://github.com/rickstaa/ros-gazebo-gym/issues/17)) ([c484443](https://github.com/rickstaa/ros-gazebo-gym/commit/c4844431d37ffa48872227c7811973bc9ca91b2c))
* remove singleton anti-pattern ([#7](https://github.com/rickstaa/ros-gazebo-gym/issues/7)) ([c2f163c](https://github.com/rickstaa/ros-gazebo-gym/commit/c2f163cd33d8d9124a10d1914a7afc368bb9008c))


### Documentation

* add pre-commit note ([#16](https://github.com/rickstaa/ros-gazebo-gym/issues/16)) ([e3c1819](https://github.com/rickstaa/ros-gazebo-gym/commit/e3c181993d37a60a56daef1caec9e33bc31eba33))
* update BLC reference to SLC ([27a098b](https://github.com/rickstaa/ros-gazebo-gym/commit/27a098bdbdd6e9dfd26ff62380c6afc5a9c0cd94))

## [1.7.0](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.6.1...v1.7.0) (2022-03-07)


### Features

* disable 'visualize' argument on save ([26ac56f](https://github.com/rickstaa/ros-gazebo-gym/commit/26ac56f6bd987e6ee7f231c849b0c4b97b63b69f))

### [1.6.1](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.6.0...v1.6.1) (2022-02-24)

## [1.6.0](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.5.3...v1.6.0) (2022-02-18)


### Features

* migrate to 'set_franka_model_configuration' service ([b6f78cd](https://github.com/rickstaa/ros-gazebo-gym/commit/b6f78cd69266d20edf644ceba5b12e60037288c1))

### [1.5.3](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.5.2...v1.5.3) (2022-02-17)


### Bug Fixes

* make singleton pickable ([857f0c4](https://github.com/rickstaa/ros-gazebo-gym/commit/857f0c43be5009211a3a0a36ee3a7e58edb13a4f))

### [1.5.2](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.5.1...v1.5.2) (2022-02-16)

### [1.5.1](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.5.0...v1.5.1) (2022-02-16)

## [1.5.0](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.4.5...v1.5.0) (2022-02-16)


### Features

* **panda:** add MoveIt init pose set option ([67602f4](https://github.com/rickstaa/ros-gazebo-gym/commit/67602f4d5f5087110c60393472ca079271bf7353))
* **panda:** improve init pose sampling code ([f3790d1](https://github.com/rickstaa/ros-gazebo-gym/commit/f3790d1717db6ca0b90ad1ddb98b507703255143))
* **panda:** improve init pose set behavoir ([f769ab1](https://github.com/rickstaa/ros-gazebo-gym/commit/f769ab116938d629a2c702c03eb84596b5b304da))


### Bug Fixes

* apply fix for 'set_model_configuration' joint limit bug ([7a991cf](https://github.com/rickstaa/ros-gazebo-gym/commit/7a991cf4bc2246383ccacf570ae8d4fe9b0c0784))
* fix collision penalty bug ([526a585](https://github.com/rickstaa/ros-gazebo-gym/commit/526a585fae03182f511d8fd997915faad4993021))
* **panda:** fix several small bugs ([71554dd](https://github.com/rickstaa/ros-gazebo-gym/commit/71554dd3035d5520fbd3bb3ce5ba0a117e74b3da))

### [1.4.5](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.4.4...v1.4.5) (2022-02-09)


### Bug Fixes

* **panda:** fix another bug that caused the panda to get stuck ([021391c](https://github.com/rickstaa/ros-gazebo-gym/commit/021391c43a52e156c955d3bf826497859fdcfac5))

### [1.4.4](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.4.3...v1.4.4) (2022-02-07)


### Bug Fixes

* **panda:** fix panda init position stuck problem ([cef461c](https://github.com/rickstaa/ros-gazebo-gym/commit/cef461c1e96f83871a868468dfa7174f860541dd))

### [1.4.3](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.4.2...v1.4.3) (2022-02-07)

### [1.4.2](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.4.1...v1.4.2) (2022-02-07)

### [1.4.1](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.4.0...v1.4.1) (2022-02-05)


### Bug Fixes

* fixes some small bugs ([9259649](https://github.com/rickstaa/ros-gazebo-gym/commit/9259649c130d479c39b7caf7adfd994cf6f099b1))

## [1.4.0](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.3.1...v1.4.0) (2022-02-05)


### Features

* improve panda rviz 'visualize' argument ([5257269](https://github.com/rickstaa/ros-gazebo-gym/commit/5257269dceb05a933bbba2328c3ff55f10479aad))

### [1.3.1](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.3.0...v1.3.1) (2022-02-05)

## [1.3.0](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.2.0...v1.3.0) (2022-02-05)


### Features

* add ability to disable panda visualization ([0a654bf](https://github.com/rickstaa/ros-gazebo-gym/commit/0a654bf00e293fc8a766e16991058186d3e487f3))

## [1.2.0](https://github.com/rickstaa/ros-gazebo-gym/compare/v1.1.10...v1.2.0) (2022-02-05)


### Features

* add roslaunch log disable arg and gazebo running warning ([131884b](https://github.com/rickstaa/ros-gazebo-gym/commit/131884be7752aae389b49f4ee928a83f02679f49))

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
