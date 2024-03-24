"""Contains several helper functions that are used in the several environments."""

import collections
import copy
import glob
import os
import re
import sys
from contextlib import contextmanager

import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatusArray
from numpy import linalg
from ros_gazebo_gym.common.euler_angles import EulerAngles
from rospy.exceptions import ROSException
from tf.transformations import euler_from_quaternion


#################################################
# Type conversion functions #####################
#################################################
def model_state_msg_2_link_state_dict(link_state_msgs):
    """Converts the a `gazebo_msgs/ModelState <https://docs.ros.org/en/jade/api/gazebo_msgs/html/msg/ModelState.html>`_
    message into a state dictionary. Contrary to the original ModelState message,
    in the model state dictionary the poses and twists are grouped per link/model.

    Args:
        link_state_msgs (:obj:`gazebo_msgs.msg.ModelState`): A ModelState message.

    Returns:
        dict: A model_state dictionary.
    """  # noqa: E501
    model_state_dict = {}
    for joint_name, position, twist in zip(
        link_state_msgs.name, link_state_msgs.pose, link_state_msgs.twist
    ):
        model_state_dict[joint_name] = {}
        model_state_dict[joint_name]["pose"] = copy.deepcopy(position)
        model_state_dict[joint_name]["twist"] = copy.deepcopy(twist)

    return model_state_dict


def pose_msg_2_pose_dict(pose_msg):
    """Create a pose dictionary ``{x, y, z, rx, ry, rz, rw}`` out of a
    `geometry_msgs.msg.Pose <https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html>`_
    message.

    Args:
        pose_msg (:obj:`geometry_msgs.msg.Pose`): A pose message

    Returns:
        dict: Dictionary that contains the pose.
    """  # noqa: E501
    pose_dict = {
        "x": pose_msg.position.x,
        "y": pose_msg.position.y,
        "z": pose_msg.position.z,
        "rx": pose_msg.orientation.x,
        "ry": pose_msg.orientation.y,
        "rz": pose_msg.orientation.z,
        "rw": pose_msg.orientation.w,
    }

    return pose_dict


#################################################
# List, dict and text manipulation functions #####
#################################################


def lower_first_char(string):
    """De-capitalize the first letter of a string.

    Args:
        string (str): The input string.

    Returns:
        str: The de-capitalized string.

    .. note::
        This function is not the exact opposite of the capitalize function of the
        standard library. For example, capitalize('abC') returns Abc rather than AbC.
    """
    if not string:  # If empty
        return

    return string[0].lower() + string[1:]


def wrap_space_around(text):
    """Wrap one additional space around text if it is not already present.

    Args:
        text (str): Text

    Returns:
        str: Text with extra spaces around it.
    """
    text = text.strip()
    return " " + text + " "


def to_pascal_case(text):
    """Convert a string to pascal case.

    Args:
        text (str): Text

    Returns:
        str: Text in pascal case.
    """
    # Split the string into words.
    words = re.findall(r"[a-zA-Z0-9]+", text)

    # Capitalize the first letter of each word and join them together.
    return "".join(word.capitalize() for word in words)


def to_snake_case(text):
    """Convert a string to snake case.

    Args:
        text (str): Text

    Returns:
        str: Text in snake case.
    """
    # Replace any capital letters with an underscore followed by the lowercase letter.
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", text)
    s2 = re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1).lower()
    return s2


def list_2_human_text(input_list, separator=", ", end_separator=" & "):
    """Function converts a list of values into human readable sentence.

    Example:
        Using this function a list of 4 items ``[item1, item2, item3, item4]`` becomes
        ``item2, item3 & item4``.

    Args:
        input_list (list): A input list.

    Returns:
        str: A human readable string that can be printed.
    """
    if isinstance(input_list, str):
        return input_list

    # Convert input to a list.
    if not isinstance(input_list, list):
        input_list = list(input_list)

    # Add spaces around separators if not present.
    separator = wrap_space_around(separator)[1:]
    end_separator = wrap_space_around(end_separator)

    # Create human readable comma delimited text.
    if len(input_list) > 1:
        return end_separator.join(
            [separator.join(map(str, input_list[:-1])), str(input_list[-1])]
        )
    elif len(input_list) == 1:
        return str(input_list[0])
    return ""


def split_dict(input_dict, *args):
    """Split a dictionary into smaller dictionaries based on the keys.

    Example:

        .. code-block:: python

            split_dict_list = split_dict(
                input_dict, ["first_dict_key1", "first_dict_key2"],
                ["second_dict_key1", "second_dict_key2"]
            )

    Args:
        input_dict (dict): Input dictionary.
        *args (list): Lists containing the keys you want to have in the successive
            dictionaries.

    Returns:
        list: A list containing the splitted dictionaries.
    """
    split_dict_list = []
    for split_list_item in args:
        split_dict_list.append(
            {key: val for key, val in input_dict.items() if key in split_list_item}
        )

    return split_dict_list


def split_bounds_dict(bounds_dict):
    """Splits the bounding region dictionary into two separate bounding dictionaries,
    one for the ``ee_pose`` and one fore the ``joint_pose``.

    Args:
        bounds_dict (dict): Original bounds dictionary.

    Returns:
        (tuple): tuple containing:

            - :obj:`dict`: ee_pose bounding region dictionary.
            - :obj:`dict`: joint_pose bounding region dictionary.
    """
    # Split bounds dict into ee_pose bounds and joint_pose bounds dictionaries.
    split_dict_list = split_dict(
        bounds_dict,
        ["x_min", "x_max", "y_min", "y_max", "z_min", "z_max"],
        [
            "panda_joint1_min",
            "panda_joint1_max",
            "panda_joint2_min",
            "panda_joint2_max",
            "panda_joint3_min",
            "panda_joint3_max",
            "panda_joint4_min",
            "panda_joint4_max",
            "panda_joint5_min",
            "panda_joint5_max",
            "panda_joint6_min",
            "panda_joint6_max",
            "panda_joint7_min",
            "panda_joint7_max",
            "panda_finger_joint1_min",
            "panda_finger_joint1_max",
            "panda_finger_joint2_min",
            "panda_finger_joint2_max",
            "gripper_width_min",
            "gripper_width_max",
        ],
    )

    return split_dict_list[0], split_dict_list[1]


def gripper_width_2_finger_joints_positions(input_dict, joints):
    """Replaces a 'gripper_width' key in a dictionary with corresponding finger joint
    position keys.

    Args:
        input_dict (dict): The dictionary that contains the 'gripper_Width'.
        joints (list): The available finger joints.

    Returns:
        dict: The new dictionary that contains the finger joint positions.
    """
    output_dict = {}
    for key, val in input_dict.items():
        if key == "gripper_width":
            for joint in joints:
                output_dict[key.replace("gripper_width", joint)] = val / 2
        else:
            output_dict[key] = val

    return output_dict


def split_pose_dict(pose_dict):
    """Splits a pose dictionary into two separate pose dictionaries, one for the
    ``ee_pose`` and one fore the ``joint_pose``.

    Args:
        bounding_region (dict): Original bounds dictionary.

    Returns:
        (tuple): tuple containing:

            - :obj:`dict`: ee_pose bounding region dictionary.
            - :obj:`dict`: joint_pose dictionary.
    """
    # Split bounds dict into ee_pose and joint_pose dictionaries.
    split_dict_list = split_dict(
        pose_dict,
        ["x", "y", "z", "rx", "ry", "rz", "rw"],
        [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
            "panda_finger_joint1",
            "panda_finger_joint2",
            "gripper_width",
        ],
    )

    return split_dict_list[0], split_dict_list[1]


def shallow_dict_merge(*args, order=None):
    """Given several dicts, merge them into a new dict as a shallow copy.

    Args:
        args (dict): The input dictionaries.
        order (list): The order in which you want to have the keys.

    Returns:
        dict: The new merged dictionary.
    """
    z = {}
    for arg in args:
        z.update(arg.copy())

    # Re-order if requested.
    if order is not None:
        order_tmp = order.copy()
        for key in z.keys():
            if key not in order_tmp:
                order_tmp.append(key)
        z = {k: z[k] for k in order_tmp}

    return z


def flatten_list(input_list):
    """Function used to flatten a list containing sublists. It does this by calling
    itself recursively.

    Args:
        input_list (list): A list containing strings or other lists.

    Returns:
        list: The flattened list.
    """
    flattened_list = []
    for list_item in input_list:
        if type(list_item) is list:
            flattened_list.extend(
                flatten_list(list_item)
            )  # NOTE: Calls itself recursively.
        else:
            flattened_list.append(list_item)

    return flattened_list


def deep_update(d, u=None, fixed=False, **kwargs):  # noqa: C901
    """Updates a dictionary recursively (i.e. deep update). This function takes a update
    dictionary and/or keyword arguments. When a keyword argument is supplied, the
    key-value pair is changed if it exists somewhere in the dictionary.

    Args:
        d (dict): Dictionary you want to update.
        u (dict, optional): The update dictionary.
        fixed (bool, optional): Whether you want the input dictionary to be fixed
            (i.e. only change keys that are already present). Defaults to ``False``.
        **kwargs: Keyword arguments used for creating the dictionary keys.

    Returns:
        dict: The updated dictionary.

    .. seealso::
        Based on the answer given by `@alex-martelli <https://stackoverflow.com/users/95810/alex-martelli>`_
        on `this stackoverflow question <https://stackoverflow.com/questions/3232943/update-value-of-a-nested-dictionary-of-varying-depth>`_.
    """  # noqa: E501
    # Update dict based on input dictionary.
    if u:
        for k, v in u.items():
            if isinstance(v, collections.abc.Mapping):
                if k in d.keys() or not fixed:
                    d[k] = deep_update(d.get(k, {}), v, fixed=fixed)
            else:
                if k in d.keys() or not fixed:
                    d[k] = v

    # Update dict based on keyword arguments.
    for key, val in kwargs.items():
        for k, v in d.items():
            if isinstance(v, collections.abc.Mapping):
                d[k] = deep_update(v, fixed=fixed, **{key: val})
            else:
                if k == key and key in d.keys():
                    d[k] = val

    # Print warning if no update dictionary or keyword argument was supplied.
    if not kwargs and not u:
        rospy.logwarn(
            "Returning original dictionary since no update dictionary or keyword "
            "argument was supplied."
        )

    return d


def is_sublist(lst1, lst2):
    """Checks whether lst1 is a sublist of lst2.

    Args:
        lst1 (list): List 1.
        lst2 (list): List 2.

    Returns:
        bool: Boolean specifying whether lst1 is a sublist of lst2.
    """
    return all(element in lst2 for element in lst1)


def remove_dict_none_values(input_dict):
    """Removes all the None values from a dictionary.

    Args:
        input_dict (dict): The input dictionary.

    Returns:
        dict: The dictionary without the None values.
    """
    if not isinstance(input_dict, dict):
        return input_dict
    return {
        k: remove_dict_none_values(v)
        for k, v in input_dict.items()
        if v is not None and remove_dict_none_values(v) is not None
    }


#################################################
# Argument validation functions #################
#################################################
def has_invalid_type(variable, variable_types, items_types=None, depth=0):  # noqa: C901
    """Validates whether a variable or its attributes has an invalid type.

    Args:
        variable (object): The variable you want to check.
        variable_types (tuple): The type the variable can have.
        items_types (tuple): The types the dictionary or list values can have.

    Returns:
        (tuple): tuple containing:

            - :obj:`bool`: A bool specifying whether a type was invalid.
            - :obj:`int`: The maximum depth at which the type was invalid.
            - :class:`type`: The types that were invalid.
    """
    # If one type was given make tuple.
    if isinstance(variable_types, type):
        variable_types = (variable_types,)
    if isinstance(items_types, type):
        items_types = (items_types,)

    # Check if variable type is valid.
    if type(variable) in variable_types:
        # Check list or dictionary value types are valid.
        if items_types:  # If items_types == None we are at the deepest level.
            if isinstance(variable, dict):
                # Check if the dictionary values are of the right type.
                depth += 1
                invalid_types = []
                for key, val in variable.items():
                    if type(val) in [dict, list]:
                        retval, depth, invalid_type = has_invalid_type(
                            val,
                            variable_types=items_types,
                            items_types=items_types,
                            depth=depth,
                        )
                    else:
                        retval, depth, invalid_type = has_invalid_type(
                            val, variable_types=items_types, depth=depth
                        )
                    if retval:  # If invalid type was found.
                        if invalid_type not in invalid_types:  # If not already in list.
                            invalid_types.append(invalid_type)
                return retval, depth, flatten_list(invalid_types)
            elif isinstance(variable, list):
                # Check if the list values are of the right type.
                depth += 1
                invalid_types = []
                for val in variable:
                    if type(val) in [dict, list]:
                        retval, depth, invalid_type = has_invalid_type(
                            val,
                            variable_types=items_types,
                            items_types=items_types,
                            depth=depth,
                        )
                    else:
                        retval, depth, invalid_type = has_invalid_type(
                            val, variable_types=items_types, depth=depth
                        )
                    if retval:  # If invalid type was found.
                        if invalid_type not in invalid_types:  # If not already in list.
                            invalid_types.append(invalid_type)
                return retval, depth, flatten_list(invalid_types)
        else:
            # Return type not invalid bool, depth and type.
            return False, depth, []

    # Return type invalid bool, depth and type.
    return True, depth, type(variable)


#################################################
# Other functions ###############################
#################################################
def action_server_exists(topic_name):
    """Checks whether a topic contains an action server
    is running.

    Args:
        topic_name (str): Action server topic name.

    Returns:
        bool: Boolean specifying whether the action service exists.
    """
    # Strip action server specific topics from topic name.
    if topic_name.split("/")[-1] in ["cancel", "feedback", "goal", "result", "status"]:
        topic_name = "/".join(topic_name.split("/")[:-1])
    if topic_name[-1] == "/":
        topic_name = topic_name[:-1]

    # Validate if action server topic exists.
    try:
        rospy.wait_for_message("%s/status" % topic_name, GoalStatusArray, timeout=5)
    except ROSException:
        return False

    # Check if topic contains action client.
    exists = False
    for item in rospy.get_published_topics():
        if "%s/status" % topic_name in item[0]:
            if "actionlib_msgs" in item[1]:
                exists = True
            else:
                exists = False

    return exists


def find_gazebo_model_path(model_name, models_directory_path, extension=""):
    """Finds the path of the ``sdf`` or ``urdf`` file that belongs to a given
    ``model_name``. This is done by searching in the ``models_directory_path`` folder.
    If no file was found the model file path is returned empty.

    Args:
        model_name (str): The name of the model for which you want to find the path.
        models_directory_path (str): The path of the folder that contains the gazebo
            models. extension (str, optional): The model path extension. Defaults to
            ``""`` meaning that the function will first look for a ``sdf`` file and if
            that is not found it will look for a ``urdf`` file.

    Returns:
        (tuple): tuple containing:

            - :obj:`str`: The path where the ``sdf`` or ``urdf`` model file can be
              found.
            - :obj:`str`: Extension of the model file.
    """
    if extension and not extension.startswith("."):
        extension = "." + extension

    # Try to find the model path for a given model_name.
    model_directory_path = os.path.join(models_directory_path, model_name)
    if os.path.isdir(model_directory_path):  # Check if model directory exists.
        if extension != "":
            model_path = glob.glob(
                os.path.join(model_directory_path, "model" + extension)
            )
            if model_path:
                return model_path[0]
        else:  # no extension given.
            for ext in [".sdf", ".urdf"]:
                model_path = glob.glob(
                    os.path.join(model_directory_path, "model" + ext)
                )
                if model_path:
                    return model_path[0], ext[1:]

    # If model path could not be found.
    rospy.logwarn(
        f"Model path for '{model_name}' could not be found. Please check if the "
        f"'{model_name}.sdf' or '{model_name}.urdf' file exist in the model directory "
        f"'{model_directory_path}'."
    )
    return "", ""


def get_orientation_euler(quaternion):
    """Converts pose (position, orientation) to euler angles.

    Args:
        quaternion (:obj:`geometry_msgs.Pose`): Input quaternion.

    Returns=:
        :obj:`~ros_gazebo_gym.common.euler_angles.EulerAngles`: Object containing the
            yaw (z), pitch (y) and roll (x) euler angles.
    """
    # Convert quaternion to euler.
    orientation_list = [
        quaternion.orientation.x,
        quaternion.orientation.y,
        quaternion.orientation.z,
        quaternion.orientation.w,
    ]
    euler_resp = euler_from_quaternion(orientation_list, "rzyx")

    # Convert list to euler object.
    euler = EulerAngles()
    euler.y = euler_resp[0]
    euler.p = euler_resp[1]
    euler.r = euler_resp[2]

    return euler


def quaternion_norm(quaternion):
    """Calculates the norm of a quaternion.

    Args:
        Quaternion (:obj:`geometry_msgs.msg.Quaternion`): A quaternion.

    Returns:
        float: The norm of the quaternion.
    """
    return linalg.norm([quaternion.x, quaternion.y, quaternion.z, quaternion.w])


def normalize_quaternion(quaternion):
    """Normalizes a given quaternion.

    Args:
        quaternion (:obj:`geometry_msgs.msg.Quaternion`): A quaternion.

    Returns:
        :obj:`geometry_msgs.msg.Quaternion`: The normalized quaternion.
    """
    quaternion = copy.deepcopy(
        quaternion
    )  # Make sure the original object is not changed.
    norm = quaternion_norm(quaternion)

    # Normalize quaternion.
    if np.isnan(norm):
        rospy.logwarn(
            "Quaternion could not be normalized since the norm could not be "
            "calculated."
        )
    elif norm == 0.0:  # Transform to identity.
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = 0.0
        quaternion.w = 1.0
    else:
        quaternion.x = quaternion.x / norm
        quaternion.y = quaternion.y / norm
        quaternion.z = quaternion.z / norm
        quaternion.w = quaternion.w / norm

    return quaternion


class DummyFile(object):
    """Dummy file class to redirect stderr to."""

    def write(self, x):
        """Writes the given string to the dummy file."""
        pass


@contextmanager
def suppress_stderr():
    """Suppresses the stderr output of a code block.

    Example:
            .. code-block:: python

                with suppress_stderr():
                    # Code block that will not print stderr.
                    sys.stderr.write("This will not be printed.")
    """
    orig_stderr = sys.stderr
    sys.stderr = DummyFile()
    try:
        yield
    finally:
        sys.stderr = orig_stderr
