#!/usr/bin/env python3
"""Contains a list of the available openai_ros gym environments.

NOTE: Here is where you have to PLACE YOUR NEW TASK ENV
"""

# NOTE: Each environment should contain a 'name', 'module' and default 'max_steps' key.
ENVS = {
    # Cartpole task envs
    "CartPoleStayUp-v0": {
        "module": "openai_ros.task_envs.cartpole_stay_up.stay_up:CartPoleStayUpEnv",
        "max_steps": 10000,
    },
    # Fetch task envs
    "FetchTest-v0": {
        "module": "openai_ros.task_envs.fetch:fetch_test_task",
        "max_steps": 10000,
    },
    "FetchSimpleTest-v0": {
        "module": "openai_ros.task_envs.fetch.fetch_simple_task:FetchSimpleTestEnv",
        "max_steps": 10000,
    },
    "FetchPickAndPlace-v0": {
        "module": "openai_ros.task_envs.fetch.fetch_pick_and_place_task:FetchPickAndPlaceEnv",
        "max_steps": 10000,
    },
    "FetchPush-v0": {
        "module": "openai_ros.task_envs.fetch.fetch_push:FetchPushEnv",
        "max_steps": 10000,
    },
    # Hopper task envs
    "HopperStayUp-v0": {
        "module": "openai_ros.task_envs.hopper.hopper_stay_up:HopperStayUpEnv",
        "max_steps": 10000,
    },
    # Husarion task envs
    "HusarionGetToPosTurtleBotPlayGround-v0": {
        "module": "openai_ros.task_envs.husarion:husarion_get_to_position_turtlebot_playground",
        "max_steps": 10000,
    },
    # IriWam task envs
    "IriWamTcpToBowl-v0": {
        "module": "openai_ros.task_envs.iriwam.tcp_to_bowl:IriWamTcpToBowlEnv",
        "max_steps": 10000,
    },
    # Moving_cube task envs
    "MovingCubeOneDiskWalk-v0": {
        "module": "openai_ros.task_envs.moving_cube:one_disk_walk.one_disk_walk:MovingCubeOneDiskWalk-v0",
        "max_steps": 10000,
    },
    # Parrot drone task envs
    "ParrotDroneGoto-v0": {
        "module": "openai_ros.task_envs.parrotdrone.parrotdrone_goto:ParrotDroneGotoEnv",
        "max_steps": 10000,
    },
    # Sawyer task envs
    "SawyerTouchCube-v0": {
        "module": "openai_ros.task_envs.sawyer.learn_to_touch_cube:SawyerTouchCubeEnv",
        "max_steps": 10000,
    },
    # Shadow hand task envs
    "ShadowTcGetBall-v0": {
        "module": "openai_ros.task_envs.shadow_tc.learn_to_pick_ball:ShadowTcGetBallEnv",
        "max_steps": 10000,
    },
    # Sumit XL room task envs
    "SumitXlRoom-v0": {
        "module": "openai_ros.task_envs.sumit_xl.sumit_xl_room:SumitXlRoom",
        "max_steps": 10000,
    },
    # Turtlebot2 task envs
    "MyTurtleBot2Maze-v0": {
        "module": "openai_ros.task_envs.turtlebot2.turtlebot2_maze:TurtleBot2MazeEnv",
        "max_steps": 10000,
    },
    "MyTurtleBot2Wall-v0": {
        "module": "openai_ros.task_envs.turtlebot2.turtlebot2_wall:TurtleBot2WallEnv",
        "max_steps": 10000,
    },
    # Turtlebot3 task envs
    "TurtleBot3World-v0": {
        "module": "openai_ros.task_envs.turtlebot3.turtlebot3_world:TurtleBot3WorldEnv",
        "max_steps": 10000,
    },
    # Wamv task envs
    "WamvNavTwoSetsBuoys-v0": {
        "module": "openai_ros.task_envs.wamv.wamv_nav_twosets_buoys:WamvNavTwoSetsBuoysEnv",
        "max_steps": 10000,
    },
}  # noqa: E501