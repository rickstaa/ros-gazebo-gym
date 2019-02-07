#!/usr/bin/env python
from gym.envs.registration import register
from gym import envs

def RegisterOpenAI_Ros_Env(task_env, timestep_limit_per_episode = 10000):
    """
    Registers all the ENVS supported in OpenAI ROS. This way we can load them
    with variable limits.
    Here is where you have to PLACE YOUR NEW TASK ENV, to be registered and accesible.
    return: False if the Task_Env wasnt registered, True if it was.
    """
    
    ###########################################################################
    # MovingCube Task-Robot Envs
    
    result = True
    
    if task_env == 'MovingCubeOneDiskWalk-v0':
        # We register the Class through the Gym system
        register(
                id='MovingCubeOneDiskWalk-v0',
                entry_point='openai_ros:task_envs.moving_cube.one_disk_walk.MovingCubeOneDiskWalkEnv',
                timestep_limit=timestep_limit_per_episode,
            )
        # We have to import the Class that we registered so that it can be found afterwards in the Make
        from openai_ros.task_envs.moving_cube import one_disk_walk
        
    # Add here your Task Envs to be registered
    else:
        result = False
    
    ###########################################################################
    
    
    
    if result:
        # We check that it was really registered
        supported_gym_envs = GetAllRegisteredGymEnvs()
        #print("REGISTERED GYM ENVS===>"+str(supported_gym_envs))
        assert (task_env in supported_gym_envs), "The Task_Robot_ENV given is not Registered ==>"+str(task_and_robot_environment_name)
    
    return result
    
def GetAllRegisteredGymEnvs():
    """
    Returns a List of all the registered Envs in the system
    return EX: ['Copy-v0', 'RepeatCopy-v0', 'ReversedAddition-v0', ... ]
    """
    
    all_envs = envs.registry.all()
    env_ids = [env_spec.id for env_spec in all_envs]

    return env_ids
        