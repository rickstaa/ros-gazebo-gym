#!/usr/bin/env python3
"""Contains a list of the available openai_ros gym environments.

NOTE: Here is where you have to PLACE YOUR NEW TASK ENV
"""

# NOTE: Each environment should contain a 'name', 'module' and default 'max_steps' key.
ENVS = {
    "MovingCubeOneDiskWalk-v0": {
        "module": "openai_ros.task_envs.moving_cube:one_disk_walk",
        "max_steps": 10000,
    },
    "HusarionGetToPosTurtleBotPlayGround-v0": {
        "module": "openai_ros.task_envs.husarion:husarion_get_to_position_turtlebot_playground",
        "max_steps": 10000,
    },
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
    "CartPoleStayUp-v0": {
        "module": "openai_ros.task_envs.cartpole_stay_up.stay_up:CartPoleStayUpEnv",
        "max_steps": 10000,
    },
    "HopperStayUp-v0": {
        "module": "openai_ros.task_envs.hopper.hopper_stay_up:HopperStayUpEnv",
        "max_steps": 10000,
    },
    "IriWamTcpToBowl-v0": {
        "module": "openai_ros.task_envs.iriwam.tcp_to_bowl:IriWamTcpToBowlEnv",
        "max_steps": 10000,
    },
    "ParrotDroneGoto-v0": {
        "module": "openai_ros.task_envs.parrotdrone.parrotdrone_goto:ParrotDroneGotoEnv",
        "max_steps": 10000,
    },
    "SawyerTouchCube-v0": {
        "module": "openai_ros.task_envs.sawyer.learn_to_touch_cube:SawyerTouchCubeEnv",
        "max_steps": 10000,
    },
    "ShadowTcGetBall-v0": {
        "module": "openai_ros.task_envs.shadow_tc.learn_to_pick_ball:ShadowTcGetBallEnv",
        "max_steps": 10000,
    },
    "SumitXlRoom-v0": {
        "module": "openai_ros.task_envs.sumit_xl.sumit_xl_room:SumitXlRoom",
        "max_steps": 10000,
    },
    "MyTurtleBot2Maze-v0": {
        "module": "openai_ros.task_envs.turtlebot2.turtlebot2_maze:TurtleBot2MazeEnv",
        "max_steps": 10000,
    },
    "MyTurtleBot2Wall-v0": {
        "module": "openai_ros.task_envs.turtlebot2.turtlebot2_wall:TurtleBot2WallEnv",
        "max_steps": 10000,
    },
    "TurtleBot3World-v0": {
        "module": "openai_ros.task_envs.turtlebot3.turtlebot3_world:TurtleBot3WorldEnv",
        "max_steps": 10000,
    },
    "WamvNavTwoSetsBuoys-v0": {
        "module": "openai_ros.task_envs.wamv.wamv_nav_twosets_buoys:WamvNavTwoSetsBuoysEnv",
        "max_steps": 10000,
    },
}
