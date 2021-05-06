#!/usr/bin/env python3
"""Contains several helper functions that are used in the openai_ros package.
"""

import os
import subprocess
import sys
from pathlib import Path

import catkin
import catkin_pkg
import pygit2
import rospkg
import rospy
import ruamel.yaml as yaml
from gym import envs
from gym.envs.registration import register

# Import a list with the available openai_ros environments
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
        source_command = ". {};".format(
            Path(workspace_path).joinpath("devel", "setup.sh")
        )
        package_command = f"rospack find {package_name}"
        command = source_command + package_command
        try:
            global_pkg_path = subprocess.check_output(
                command, shell=True, text=True, stderr=subprocess.DEVNULL
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
    pathstr = Path(workspace_path).joinpath("src", "rosdeps", package_name)
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


def build_catkin_ws(workspace_path):
    """Re-builds a catkin workspace.

    Args:
        workspace_path (str): The path of the catkin workspace.

    Raises:
        Exception: When something goes wrong while re-building the workspace.
    """
    catkin_make_used = os.path.exists(workspace_path + "/.catkin_workspace")

    # Build workspace
    if catkin_make_used:  # Use catkin_make
        rosbuild_command = "catkin_make"
    else:  # Use catkin build
        rosbuild_command = "catkin build"
    rosbuild_command = "catkin build"
    p = subprocess.call(rosbuild_command, shell=True)

    # Catch result
    if p != 0:
        raise Exception(
            "Something went wrong while trying to build the catkin workspace."
        )


def package_installer(package_name, workspace_path=None):
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

    # Retrieve package paths
    global_pkg_path = get_global_pkg_path(package_name)
    local_pkg_path = get_local_pkg_path(package_name, workspace_path)

    # Download the package repository if it is not installed in the right location
    package_installed = True
    deps_cloned = False
    if ROSDEP_INDEX and package_name in ROSDEP_INDEX.keys():
        if not local_pkg_path and (
            not global_pkg_path
            or (
                global_pkg_path
                and (
                    not global_pkg_path != local_pkg_path
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
            rospy.logwarn(debug_message)

            # Download package repository
            try:
                clone_dependency_repo(
                    package_name,
                    workspace_path,
                    ROSDEP_INDEX[package_name]["git"],
                    ROSDEP_INDEX[package_name]["branch"],
                )
                deps_cloned = True
            except Exception:
                package_installed = False
                warn_msg = f"ROS dependency '{package_name}' could not be installed."
                rospy.logwarn(warn_msg)

            # Download additional dependencies
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
                                not global_dep_pkg_path != local_dep_pkg_path
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
                        rospy.logwarn(debug_message)
                        try:
                            clone_dependency_repo(
                                dep,
                                workspace_path,
                                dep_index_info["git"],
                                dep_index_info["branch"],
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
        rospy.loginfo("Re-building catkin workspace since new packages were added.")
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
