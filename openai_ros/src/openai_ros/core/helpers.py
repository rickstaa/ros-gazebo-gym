#!/usr/bin/env python3
"""Contains several helper functions that are used in the openai_ros package.
"""

import os
import subprocess
import sys
from pathlib import Path

import catkin
import catkin_pkg
import numpy as np
import pygit2
import rosparam
import rospkg
import rospy
import ruamel.yaml as yaml
from gym import envs
from gym.envs.registration import register
from openai_ros.task_envs.task_envs_list import ENVS
from tqdm import tqdm

# Dependency index
ROSDEP_INDEX_PATH = "../cfg/rosdep.yaml"
ROSDEP_INDEX = None


class GitProgressCallback(pygit2.RemoteCallbacks):
    """Callback class that can be used to overwrite the transfer_progress callback."""

    def __init__(self):
        super().__init__()
        self.pbar = tqdm()

    def transfer_progress(self, statsTransferProgress):
        """Displays the current git transfer progress.

        Args:
            statsTransferProgress (pygit2.statsTransferProgress): The progress up to
                now.
        """
        self.pbar.total = statsTransferProgress.total_objects
        self.pbar.n = statsTransferProgress.received_objects
        self.pbar.refresh()


def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
            It must be "yes" (the default), "no" or None (meaning
            an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".

    Args:
        question (str): String presented to the user.
        default (str, optional): The presumed answer if the user just hits <Enter>. It
            must be "yes" (the default), "no" or None (meaning an answer is required of
            the user). Defaults to "yes".

    Raises:
        ValueError: If the default answer is not correct.

    Returns:
        str: Returns the given answer.

    .. seealso::
        This function was based on the function given by `@fmark <https://stackoverflow.com/users/103225/fmark>`_ in
        `this stackoverflow question <https://stackoverflow.com/questions/3041986/apt-command-line-interface-like-yes-no-input>`_
    """  # noqa: E501
    valid = {"yes": True, "y": True, "ye": True, "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = input().lower()
        if default is not None and choice == "":
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' " "(or 'y' or 'n').\n")


def register_openai_ros_env(task_env, max_episode_steps=10000):
    """Register a given openai_ros task environment.

    Args:
        task_env (str): The openai_ros task environment you want to register.
        max_episode_steps (int, optional): The max episode step you want to set for the
            environment. Defaults to 10000.

    Raises:
        Exception: When something went wrong during the registration.
    """
    if task_env in ENVS.keys():
        # Register gym environment
        max_episode_steps = (
            max_episode_steps if max_episode_steps else ENVS[task_env]["max_steps"]
        )
        try:
            register(
                id=task_env,
                entry_point=ENVS[task_env]["module"],
                max_episode_steps=max_episode_steps,
            )
        except Exception:
            raise Exception(
                f"Something went wrong while trying to register the '{task_env}' gym "
                "environment."
            )
    else:
        env_not_found_msg = (
            f"Gym environment '{task_env}' could not be registered. As it is not "
            "implemented in the 'openai_ros' package. Implemented environment are:"
        )
        for key in ENVS:
            env_not_found_msg += f"\t\n - {key}"
        raise Exception(env_not_found_msg)

    # Double check if the environment was really registered
    assert task_env in get_registered_gym_envs(), (
        f"Something went wrong while trying to register the '{task_env}' gym "
        "environment."
    )


def get_registered_gym_envs():
    """Retrieve all currently registered gym environments.

    Returns:
        list: List with all the gym environments that are registered.
    """
    return [env_spec.id for env_spec in envs.registry.all()]


def get_global_pkg_path(package_name, workspace_path=None):
    """Retrieves the global package path. Meaning the path of a package if it is
    contained in the global ROS workspace. Returns ``None`` if the package is not
    found.

    Args:
        package_name (str): The name of the package you want to check.

    Returns:
        str: The global package path.
    """
    rp = rospkg.RosPack()
    try:
        global_pkg_path = rp.get_path(package_name)
        rospy.logdebug(
            f"Package '{package_name}' found in your global catkin workspace."
        )
    except Exception:
        # Try to source the catkin_ws if no path was found
        workspace_path = (
            workspace_path
            if workspace_path
            else catkin.workspace.get_workspaces()[0].replace("/devel", "")
        )
        # NOTE: Bash prefix needed since sourcing setup.sh doesn't seem to work
        bash_prefix = '/bin/bash -c "'
        source_command = ". {}{};".format(
            workspace_path, Path("/devel/setup.bash").resolve()
        )
        package_command = f"rospack find {package_name}"
        command = bash_prefix + source_command + package_command + '"'
        try:
            global_pkg_path = subprocess.check_output(
                command,
                shell=True,
                text=True,
                stderr=subprocess.DEVNULL,
                cwd=workspace_path,
            ).split("\n")[0]
        except Exception:
            global_pkg_path = None
    return global_pkg_path


def get_local_pkg_path(package_name, catkin_workspace):
    """Retrieves the local package path. Meaning the path of a package in the catkin
    workspace. Returns ``None`` if package is not found in the catkin workspace.

    Args:
        package_name (str): The name of the package you want to check.
        catkin_workspace (str): The catkin_workspace you want to check.

    Returns:
        str: The local package path.
    """
    local_pkg_path = [
        key
        for key, val in catkin_pkg.packages.find_packages(catkin_workspace).items()
        if val.name == package_name
    ]
    if local_pkg_path:
        local_pkg_path = str(
            Path(catkin_workspace).joinpath(local_pkg_path[0]).resolve()
        )
        rospy.logdebug(f"Package '{package_name}' found in local catkin workspace.")
    else:
        local_pkg_path = None
    return local_pkg_path


def clone_dependency_repo(package_name, workspace_path, git_src, branch=None):
    """Clones the repository of the dependency.

    Args:
        package_name (str): The package for which you want to clone the repository.
        workspace_path (str): The workspace in which you want to clone the repository.
        rosdep_index (dict): The openai_ros dependency index dictionary.
        git_src (str): The git repository url.
        branch(str, optional): The branch to checkout. Defaults to ``None``.
    """
    pathstr = str(Path(workspace_path).joinpath("src", "rosdeps", package_name))
    try:
        pygit2.clone_repository(
            git_src,
            pathstr,
            checkout_branch=branch,
            callbacks=GitProgressCallback(),
        )
    except Exception as e:
        rospy.logwarn(
            f"Could no clone the '{package_name}' package repository as {e.args[0]}."
        )
        raise e


def build_catkin_ws(workspace_path, install_ros_deps=True):
    """Installs the system dependencies and re-builds a catkin workspace.

    Args:
        workspace_path (str): The path of the catkin workspace.
        install_ros_deps (bool, optional): Whether you also want to installt he system
            using rosdep. Defaults to ``True``.

    Raises:
        Exception: When something goes wrong while re-building the workspace.
    """
    catkin_make_used = os.path.exists(workspace_path + "/.catkin_workspace")

    # Install system dependencies using rosdep
    if install_ros_deps:
        rospy.logwarn(
            "Several system dependencies are required to use the newly installed ROS "
            "packages."
        )
        answer = query_yes_no(
            "Do you want to install these ROS system dependencies?", default="no"
        )
        if answer:
            rospy.logwarn(
                "Installing ROS system dependencies. If asked please supply your root "
                "password:"
            )
            rosdep_command = (
                f"sudo -S rosdep install --from-paths {workspace_path}/src "
                + "--ignore-src -r -y --rosdistro {}".format(
                    os.environ.get("ROS_DISTRO")
                )
            )
            p = subprocess.call(rosdep_command, shell=True, cwd=workspace_path)

    # Build workspace
    if catkin_make_used:  # Use catkin_make
        rosbuild_command = "catkin_make"
        rosbuild_clean_command = "rm -r devel logs build -y"
    else:  # Use catkin build
        rosbuild_command = "catkin build"
        rosbuild_clean_command = "catkin clean -y"
    rosbuild_command = "catkin build"
    rospy.logwarn("Re-building catkin workspace.")
    p = subprocess.call(rosbuild_command, shell=True, cwd=workspace_path)

    # Catch result, clean workspace and try again on fail
    if p != 0:
        # Clean the workspace and try one more time
        rospy.logwarn(
            "Something went wrong while trying to build the catkin workspace."
        )
        answer = query_yes_no(
            "Do you want to clean the catkin workspace and try again?", default="yes"
        )
        if answer:
            rospy.logwarn("Cleaning the catkin workspace.")
            p = subprocess.call(rosbuild_clean_command, shell=True, cwd=workspace_path)
            if p != 0:
                rospy.logwarn(
                    "Something went wrong while trying to clean the catkin workspace."
                )
            else:
                rospy.logwarn("Re-building catkin workspace.")
                p = subprocess.call(rosbuild_command, shell=True, cwd=workspace_path)

        # Throw warning if something went wrong
        if p != 0:
            raise Exception(
                "Something went wrong while trying to build the catkin workspace."
            )


def package_installer(package_name, workspace_path=None):  # noqa: C901
    """Install a given ROS package together with it's dependencies. This function checks
    if a ROS packages is installed and installs it if this is not the case. It uses the
    openai_ros package dependency index to clone the package and dependencies in the
    local catkin workspace and subsequently re-builds this workspace.

    Args:
        package_name (str): The package name you want to have installed.
        workspace_path (str, optional): The catkin workspace path. Defaults to ``None``
            (i.e. path will be determined).

    Returns:
        bool: Whether the package was successfully installed.
    """
    rospy.logdebug(
        f"Checking if all ROS dependencies for package '{package_name}' are installed."
    )

    # Load ROS dependency index
    global ROSDEP_INDEX
    if not ROSDEP_INDEX:
        rosdep_index_abs = Path(__file__).parent.joinpath(ROSDEP_INDEX_PATH).resolve()
        try:
            with open(rosdep_index_abs) as stream:
                ROSDEP_INDEX = yaml.safe_load(stream)
        except Exception:
            warn_msg = (
                "ROS dependencies could not be installed as something went wrong while "
                "trying to load the openai_ros index configuration file at "
                f"{rosdep_index_abs}. Please check the openai_ros index configuration "
                "file and try again."
            )
            rospy.logwarn(warn_msg)

    # Retrieve workspace path
    workspace_path = (
        workspace_path
        if workspace_path
        else catkin.workspace.get_workspaces()[0].replace("/devel", "")
    )
    if not workspace_path:
        rospy.logerr(
            "Workspace path could not be found. Please make sure that you source "
            "the workspace before calling the package_installer function or supply the "
            "function with a workspace_path."
        )
        rospy.signal_shutdown("Workspace path could not be found.")
        sys.exit(0)

    # Retrieve package paths
    global_pkg_path = get_global_pkg_path(package_name)
    local_pkg_path = get_local_pkg_path(package_name, workspace_path)

    # Download the package repository if it is not installed in the right location
    package_installed = True
    deps_cloned = False
    rospy.loginfo("Checking if all required ROS dependencies are installed...")
    if ROSDEP_INDEX and package_name in ROSDEP_INDEX.keys():
        # Download package
        if not local_pkg_path and (
            not global_pkg_path
            or (
                global_pkg_path
                and (
                    global_pkg_path != local_pkg_path
                    and not ROSDEP_INDEX[package_name]["binary"]
                )
            )
        ):
            debug_message = f"Cloning '{package_name}' into local catkin workspace."
            if (
                global_pkg_path != local_pkg_path
                and not ROSDEP_INDEX[package_name]["binary"]
            ):
                debug_message = (
                    f"Package '{package_name}' should be installed in the local catkin "
                    "workspace. " + debug_message
                )
            else:
                debug_message = (
                    f"ROS dependency '{package_name}' not installed. " + debug_message
                )
            rospy.logwarn(debug_message)

            # Download package repository
            try:
                clone_dependency_repo(
                    package_name,
                    workspace_path,
                    ROSDEP_INDEX[package_name]["git"],
                    ROSDEP_INDEX[package_name]["branch"]
                    if "branch" in ROSDEP_INDEX[package_name].keys()
                    else "main",
                )
                deps_cloned = True
            except Exception:
                package_installed = False
                warn_msg = f"ROS dependency '{package_name}' could not be installed."
                rospy.logwarn(warn_msg)

        # Download additional package dependencies
        if "deps" in ROSDEP_INDEX[package_name].keys() and isinstance(
            ROSDEP_INDEX[package_name]["deps"], dict
        ):
            for dep, dep_index_info in ROSDEP_INDEX[package_name]["deps"].items():
                global_dep_pkg_path = get_global_pkg_path(dep)
                local_dep_pkg_path = get_local_pkg_path(dep, workspace_path)
                if not local_dep_pkg_path and (
                    not global_dep_pkg_path
                    or (
                        global_dep_pkg_path
                        and (
                            global_dep_pkg_path != local_dep_pkg_path
                            and not dep_index_info["binary"]
                        )
                    )
                ):
                    debug_message = f"Cloning '{dep}' into local catkin workspace."
                    if (
                        global_dep_pkg_path != local_dep_pkg_path
                        and not dep_index_info["binary"]
                    ):
                        debug_message = (
                            f"Package '{dep}' should be installed in the local "
                            "catkin workspace. " + debug_message
                        )
                    else:
                        debug_message = (
                            f"ROS dependency '{dep}' not installed. " + debug_message
                        )
                    rospy.logwarn(debug_message)
                    try:
                        clone_dependency_repo(
                            dep,
                            workspace_path,
                            dep_index_info["git"],
                            dep_index_info["branch"]
                            if "branch" in dep_index_info.keys()
                            else "main",
                        )
                        deps_cloned = True
                    except Exception:
                        warn_msg = f"ROS dependency '{dep}' could not be installed."
                        rospy.logwarn(warn_msg)
    else:
        if global_pkg_path or local_pkg_path:
            space_str = (
                "global and local ROS workspaces"
                if global_pkg_path and local_pkg_path
                else (
                    "global ROS workspace"
                    if global_pkg_path
                    else "local catkin workspace"
                )
            )
            rospy.logdebug(
                f"Package '{package_name}' was not found in the openai_ros dependency "
                f"it however appears to be installed in the {space_str}."
            )
            package_installed = True
        else:
            rospy.logwarn(
                f"Package '{package_name}' is not installed and not present in the "
                "openai_ros dependency index. As a result it was not installed."
            )

    # Build the catkin workspace
    if deps_cloned:
        rospy.logwarn("Re-building catkin workspace since new packages were added.")
        try:
            build_catkin_ws(workspace_path)
        except Exception:
            rospy.logerr(
                "Something went wrong while trying to re-build the catkin workspace. "
                "Please build the catkin workspace manually and try again."
            )
            rospy.signal_shutdown("Shutting down ROS launch file.")
            sys.exit(0)

    # Return package path
    return package_installed


def load_ros_params_from_yaml(
    package_name, rel_path_from_package_to_file, yaml_file_name
):
    """Loads ros parameters from yaml file.

    Args:
        package_name (str): The package name that contains the configuration file.
        rel_path_from_package_to_file (str): The relative path from this package to the
            configuration file.
        yaml_file_name (str): The configuration file name.
    """
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(package_name)
    config_dir = os.path.join(pkg_path, rel_path_from_package_to_file)
    path_config_file = os.path.join(config_dir, yaml_file_name)
    paramlist = rosparam.load_file(path_config_file)
    for params, ns in paramlist:
        rosparam.upload_params(ns, params)


def get_vector_magnitude(vector):
    """Calculates the force magnitude.

    Args:
        vector (numpy.ndarray): The input vector. [description]

    Returns:
        float: The magnitude of the vector.
    """
    contact_force_np = np.array((vector.x, vector.y, vector.z))
    force_magnitude = np.linalg.norm(contact_force_np)
    return force_magnitude
