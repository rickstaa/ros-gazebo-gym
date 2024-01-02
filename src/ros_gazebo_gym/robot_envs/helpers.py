"""Contains several helper functions that are used in the robot environments."""


def remove_gripper_commands_from_joint_commands_msg(joint_commands_msg):
    """Removes the gripper commands from the joint commands message.

    Args:
        joint_commands_msg (:obj:`panda_gazebo.msg.JointCommands`): The joint
            commands message.
    """
    joint_commands = dict(
        zip(joint_commands_msg.joint_names, joint_commands_msg.joint_commands)
    )
    joint_commands.pop("gripper_width", None)
    joint_commands.pop("gripper_max_effort", None)
    joint_commands_msg.joint_names = list(joint_commands.keys())
    joint_commands_msg.joint_commands = list(joint_commands.values())
    return joint_commands_msg
