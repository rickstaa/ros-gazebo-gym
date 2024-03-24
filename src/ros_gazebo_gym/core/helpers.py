"""Contains several helper functions that are used by the
:ros-gazebo-gym:`ros_gazebo_gym <>` package for the setup of the ROS Gazebo Gym
gymnasium environments.
"""

import atexit
import os
import subprocess
import sys
import threading
import time
from io import TextIOWrapper
from pathlib import Path

import catkin
import catkin_pkg
import psutil
import pygit2
import rosparam
import rospkg
import rospy
from ruamel.yaml import YAML, YAMLError
from ros_gazebo_gym.common.helpers import remove_dict_none_values
from tqdm import tqdm

# Script settings.
ROSDEP_INDEX_PATH = "../cfg/rosdep.yaml"


class GitProgressCallback(pygit2.RemoteCallbacks):
    """A :class:`pygit2.RemoteCallbacks` callback that shows a git clone progress bar."""

    def __init__(self):
        """Initializes the :class:`GitProgressCallback` class."""
        super().__init__()
        self.pbar = tqdm(file=sys.stdout)

    def transfer_progress(self, statsTransferProgress):
        """Displays the current git transfer progress.

        Args:
            statsTransferProgress (pygit2.statsTransferProgress): The progress up to
                now.
        """
        self.pbar.total = statsTransferProgress.total_objects
        self.pbar.n = statsTransferProgress.received_objects
        self.pbar.refresh()


class PopenAutoCleanup(subprocess.Popen):
    """A :class:`subprocess.Popen` that cleans up after itself and terminates the
    process when the main Python script exits. It also allows users to specify whether
    the process is critical and show an error message when the process is no longer
    running.

    Attributes:
        critical (bool): Whether the process is critical and an error message should be
            shown when the process is no longer running.
    """

    def __init__(self, *args, critical=False, **kwargs):
        """Initializes the :class:`PopenAutoCleanup` class.

        Args:
            *args: The positional arguments that are passed to the
                :class:`subprocess.Popen` class.
            critical (bool, optional): Whether the process is critical and an error
                message should be shown when the process is no longer running.
            **kwargs: The keyword arguments that are passed to the
                :class:`subprocess.Popen` class.
        """
        self._cmd = args[0]
        super().__init__(*args, **kwargs)
        atexit.register(self._process_cleanup)  # Ensure cleanup when script exits.

        # Retrieve log file path.
        self._log_file = kwargs.get("stderr", None)
        if isinstance(self._log_file, TextIOWrapper):
            self._log_file = kwargs["stderr"].name

        # If critical, check if process is still running and show error message if not.
        if critical:
            self.critical = critical
            self._critical_check_event = threading.Event()
            self._thread = threading.Thread(target=self._critical_check, daemon=True)
            self._thread.start()

    def _process_cleanup(self):
        """Cleans up the process when the script exits."""
        rospy.logwarn(f"Cleaning ROS launch process: {self._cmd}")
        self._stop_critical_check()
        if self.poll() is None:
            self._terminate_child_processes()
            self.terminate()
            self.wait()

    def _terminate_child_processes(self):
        """Terminates all child processes of the process."""
        if not psutil.pid_exists(self.pid):
            return

        # Terminate child processes.
        parent = psutil.Process(self.pid)
        for child in parent.children(recursive=True):
            if psutil.pid_exists(child.pid):
                child.terminate()
        psutil.wait_procs(parent.children(), timeout=5)

    def _critical_check(self):
        """Checks if the process is still running and exits the script if it is not and
        the process is critical.
        """
        while not self._critical_check_event.is_set() and self.poll() is None:
            time.sleep(0.1)
        if self.critical and not self._critical_check_event.is_set():
            if self._log_file is None:
                rospy.logerr(
                    "A critical subprocess has exited unexpectedly. Check the stdout "
                    "for more information."
                )
            else:
                rospy.logerr(
                    "A critical subprocess has exited unexpectedly. Check the log "
                    f"file for more information (i.e. '{self._log_file})."
                )

    def _stop_critical_check(self):
        """Stops the critical check thread."""
        self._critical_check_event.set()

    def terminate(self):
        """Terminates the process and stops the critical check thread."""
        self._stop_critical_check()
        if self.poll() is None:
            self._terminate_child_processes()
            super().terminate()
            super().wait()

    def kill(self):
        """Kills the process and stops the critical check thread."""
        self._stop_critical_check()
        if self.poll() is None:
            self._terminate_child_processes()
            super().kill()
            super().wait()


def get_catkin_workspace_path():
    """Retrieve the path of the currently sourced catkin workspace.

    Returns:
        str: The path of the currently sourced catkin workspace. Returns ``None`` if no
            catkin workspace was sourced.
    """
    catkin_ws_path = Path(catkin.workspace.get_workspaces()[0])
    ws_path = (
        catkin_ws_path.parent if catkin_ws_path.parts[-1] == "devel" else catkin_ws_path
    )
    if not ws_path:
        return None
    return str(ws_path)


def query_yes_no(question, default="yes"):
    """Ask the user a yes/no question via raw_input() and return the answer.

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
    default = default.lower() if isinstance(default, str) else None
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    # Loop until a valid answer is given.
    while True:
        sys.stdout.write(question + prompt)
        choice = input().lower()
        if default is not None and choice == "":
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' " "(or 'y' or 'n').\n")


def get_global_pkg_path(package_name, workspace_path=None):
    """Retrieves the global package path. Meaning the path of a package if it is
    contained in the global ROS workspace. Returns ``None`` if the package is not
    found.

    Args:
        package_name (str): The name of the package you want to check.

    Returns:
        str: The global package path.
    """
    rospack = rospkg.RosPack()
    try:  # Try to retrieve workspace path.
        global_pkg_path = rospack.get_path(package_name)
        rospy.logdebug(
            f"Package '{package_name}' found in your global catkin workspace."
        )
    except Exception:  # Try to source the catkin workspace and try again.
        if not workspace_path:
            workspace_path = get_catkin_workspace_path()
        try:
            global_pkg_path = subprocess.run(
                f". ./devel/setup.bash;rospack find {package_name}",
                executable="/bin/bash",
                capture_output=True,
                shell=True,
                cwd=workspace_path,
                text=True,
            ).stdout.split("\n")[0]
            global_pkg_path = None if global_pkg_path == "" else global_pkg_path
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


def clone_repo(path, git_src, branch=None, recursive=False):
    """Clones the repository of the dependency.

    Args:
        path (str): The path to which you want to clone the repository.
        git_src (str): The git repository url.
        branch(str, optional): The branch to checkout. Defaults to ``None``.
        recursive(bool, optional): After the clone is created, initialize and clone
            submodules within based on the provided pathspec. Defaults to ``False``.
    """
    try:
        rospy.logdebug(f"Cloning '{git_src}' into '{path}'.")
        repo = pygit2.clone_repository(
            git_src,
            path,
            checkout_branch=branch,
            callbacks=GitProgressCallback(),
        )
        if recursive:
            rospy.logdebug("Pulling submodules.")
            repo.init_submodules()
            repo.update_submodules()
    except Exception as e:
        repo_name = git_src.split("/")[-1].split(".")[0]
        rospy.logwarn(
            f"Could no clone the '{repo_name}' package repository as {e.args[0]}."
        )
        raise e


def is_repo_up_to_date(repo_path):
    """Checks if the local repository is up to date with the remote repository.

    Args:
        repo_path (str): The path of the local repository.

    Returns:
        bool: Whether the local repository is up to date with the remote repository.
    """
    try:
        # Open the repository.
        repo = pygit2.Repository(repo_path)

        # Get the current commit hash of the local repository.
        local_commit_hash = repo.head.target

        # Get the latest commit hash of the remote repository.
        repo.remotes["origin"].fetch()
        latest_remote_commit = repo.revparse_single("FETCH_HEAD").id

        # Compare the commit hashes and return the result.
        return local_commit_hash.hex == latest_remote_commit.hex

    except pygit2.GitError as e:
        print(f"Error: {e}")
        return False


def ros_exit_gracefully(shutdown_msg=None, exit_code=0):
    """Shuts down the ROS node wait until it is shutdown and exits the script.

    Args:
        shutdown_msg (str, optional): The shutdown message. Defaults to ``None``.
        exit_code (int, optional): The exit code. Defaults to ``0``.
    """
    if exit_code == 0:
        rospy.loginfo(shutdown_msg)
    else:
        rospy.logerr(shutdown_msg)
    rospy.signal_shutdown(shutdown_msg)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
    sys.exit(exit_code)


def build_catkin_ws(workspace_path, install_ros_deps=False):
    """Builds the catkin workspace and installs system dependencies while doing so if
    requested.

    Args:
        workspace_path (str): The path of the catkin workspace.
        install_ros_deps (bool, optional): Whether you also want to install the system
            dependencies using rosdep before building the workspace. Defaults to
            ``False``.

    Raises:
        Exception: When something goes wrong while re-building the workspace.
    """
    catkin_make_used = os.path.exists(workspace_path + "/.catkin_workspace")

    # Install system dependencies using rosdep.
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
                "Installing ROS system dependencies using rosdep. If asked please "
                "supply your root password."
            )
            rosdep_cmd = (
                "sudo -S rosdep install --from-path src --ignore-src -r -y "
                "--rosdistro {}".format(os.environ.get("ROS_DISTRO"))
            )

            # Try to install the system dependencies.
            rosdep_install = subprocess.run(rosdep_cmd, shell=True, cwd=workspace_path)

            # Check exit code and try again if error is known.
            if rosdep_install.returncode:
                rospy.logwarn(
                    "Something went wrong while trying to install the system "
                    "dependencies. Trying again while first updating rosdep as the "
                    "root user."
                )
                rosdep_sudo_update = subprocess.run(
                    "sudo -S rosdep update", shell=True, cwd=workspace_path
                )
                rosdep_install = subprocess.run(
                    rosdep_cmd, shell=True, cwd=workspace_path
                )
                rospy.logwarn("Repairing permissions...")
                rosdep_repair = subprocess.run(
                    "sudo -S rosdep fix-permissions", shell=True, cwd=workspace_path
                )
                rospy.logwarn("Updating rosdep...")
                rosdep_update = subprocess.run(
                    "rosdep update", shell=True, cwd=workspace_path
                )

                # Throw error if something went wrong.
                if (
                    rosdep_sudo_update.returncode
                    | rosdep_install.returncode
                    | rosdep_repair.returncode
                    | rosdep_update.returncode
                ):
                    rospy.logerr(
                        "System dependencies could not be installed automatically. "
                        "Please run:\n\n\t rosdep install --from-path src --ignore-src "
                        "-r -y\n\nin the catkin workspace and try again (i.e. "
                        f"'{workspace_path}')."
                    )
                    ros_exit_gracefully(
                        shutdown_msg=f"Shutting down {rospy.get_name()}", exit_code=1
                    )

    # Build workspace.
    if catkin_make_used:
        rosbuild_cmd = "catkin_make"
        rosbuild_clean_cmd = "rm -r devel logs build -y"
    else:
        rosbuild_cmd = "catkin build"
        rosbuild_clean_cmd = "catkin clean -y"
    rosbuild_cmd = "catkin build"
    rospy.logwarn("Re-building catkin workspace.")
    rosbuild = subprocess.run(rosbuild_cmd, shell=True, cwd=workspace_path)

    # Catch result, clean workspace and try again on fail.
    if rosbuild.returncode != 0:
        # Clean the workspace and try one more time.
        rospy.logwarn(
            "Something went wrong while trying to build the catkin workspace."
        )
        answer = query_yes_no(
            "Do you want to clean the catkin workspace and try again?", default="yes"
        )
        if answer:
            rospy.logwarn("Cleaning the catkin workspace.")
            rosbuild_clean = subprocess.run(
                rosbuild_clean_cmd, shell=True, cwd=workspace_path
            )
            if rosbuild_clean.returncode != 0:
                rospy.logwarn(
                    "Something went wrong while trying to clean the catkin workspace."
                )
            else:
                rospy.logwarn("Re-building catkin workspace.")
                rosbuild = subprocess.run(rosbuild_cmd, shell=True, cwd=workspace_path)

        # Throw warning if something went wrong.
        if rosbuild.returncode != 0:
            raise Exception(
                "Something went wrong while trying to build the catkin workspace."
            )


def install_package(  # noqa: C901
    package_name, workspace_path=None, outdated_warning=True
):
    """Install a given ROS package together with it's dependencies.

    This function checks if a ROS package is installed and installs it if this is not
    the case. It uses the :ros-gazebo-gym:`ros_gazebo_gym <>` package dependency index to
    clone the package and dependencies in the local catkin workspace and subsequently
    rebuilds this workspace. It also throws a warning if a package is installed but not
    up to date.

    Args:
        package_name (str): The package you want to install.
        workspace_path (str, optional): The catkin workspace path. Defaults to ``None``
            (i.e. path will be determined).
        outdated_warning (bool, optional): Whether to show a update warning when the
            package is outdated. Defaults to ``True``.

    Returns:
        bool: Whether the package and its dependencies are installed.
    """
    rospy.logdebug(
        f"Checking if all ROS dependencies for package '{package_name}' are present."
    )

    # Retrieve workspace path.
    if not workspace_path:
        workspace_path = get_catkin_workspace_path()
    if not workspace_path:
        rospy.logerr(
            "Workspace path could not be found. Please make sure that you source "
            "the workspace before calling the 'install_package' function or supply the "
            "function with a workspace_path."
        )
        ros_exit_gracefully(
            shutdown_msg=f"Shutting down {rospy.get_name()}", exit_code=1
        )

    # Check if package exists in the global and local catkin workspaces.
    rospy.loginfo(f"Checking if '{package_name}' and its dependencies are installed.")
    global_pkg_path = get_global_pkg_path(package_name)
    local_pkg_path = get_local_pkg_path(package_name, workspace_path)

    # Load ROS dependency index.
    rosdep_index_path = Path(__file__).parent.joinpath(ROSDEP_INDEX_PATH).resolve()
    try:
        with open(rosdep_index_path) as stream:
            rosdep_index = YAML(typ="safe").load(stream)
    except YAMLError:
        warn_msg = (
            f"Could not check whether '{package_name}' and its dependencies are "
            "installed since something went wrong while trying to load the "
            f"'ros_gazebo_gym' index configuration file (i.e. '{rosdep_index_path}'). "
            "Please check the 'ros_gazebo_gym' index configuration file."
        )
        rospy.logwarn(warn_msg)

    # Download the package repository and its dependencies if it is not present.
    deps_cloned = False
    if rosdep_index and package_name in rosdep_index.keys():
        package_clone_path = str(
            Path(workspace_path).joinpath("src", "rosdeps", package_name)
        )
        # Clone package if it is not present in the local ROS workspace.
        if not local_pkg_path and (
            not global_pkg_path
            or (
                global_pkg_path
                and (
                    global_pkg_path != local_pkg_path
                    and not rosdep_index[package_name]["binary"]
                )  # Binary packages are allowed to be installed globally.
            )
        ):
            # Display warn message.
            warn_message = f"Cloning '{package_name}' into local catkin workspace."
            if (
                global_pkg_path != local_pkg_path
                and not rosdep_index[package_name]["binary"]
            ):
                warn_message = (
                    f"Package '{package_name}' is not set as a binary package in the "
                    f"'ros_gazebo_gym' package index (i.e. 'rosdep_index_path') and "
                    "should therefore be installed in the local catkin workspace. "
                    + warn_message
                )
            else:
                warn_message = (
                    f"ROS dependency '{package_name}' not installed. " + warn_message
                )
            rospy.logwarn(warn_message)

            # Download package repository.
            try:
                clone_repo(
                    package_clone_path,
                    rosdep_index[package_name]["git_url"],
                    branch=(
                        rosdep_index[package_name]["git_branch"]
                        if "git_branch" in rosdep_index[package_name].keys()
                        else "main"
                    ),
                    recursive=True,
                )
                deps_cloned = True
            except Exception:
                warn_msg = f"ROS dependency '{package_name}' could not be cloned."
                rospy.logwarn(warn_msg)
                return False
        else:
            # Throw warning if package is installed but not up to date.
            if outdated_warning and not is_repo_up_to_date(package_clone_path):
                rospy.logwarn(
                    f"Package '{package_name}' is installed but not up to date. "
                    "If you want to have the latest version of the package, please "
                    "update it manually by running 'git pull' in the package directory "
                    f"(i.e. '{package_clone_path}')."
                )

        # Download additional package dependencies.
        if "deps" in rosdep_index[package_name].keys() and isinstance(
            rosdep_index[package_name]["deps"], dict
        ):
            for dep, dep_index_info in rosdep_index[package_name]["deps"].items():
                global_dep_pkg_path = get_global_pkg_path(dep)
                local_dep_pkg_path = get_local_pkg_path(dep, workspace_path)
                dep_clone_path = str(
                    Path(workspace_path).joinpath("src", "rosdeps", dep)
                )
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
                    warn_message = f"Cloning '{dep}' into local catkin workspace."
                    if (
                        global_dep_pkg_path != local_dep_pkg_path
                        and not dep_index_info["binary"]
                    ):
                        warn_message = (
                            f"Package '{dep}' should be installed in the local "
                            "catkin workspace. " + warn_message
                        )
                    else:
                        warn_message = (
                            f"ROS dependency '{dep}' not installed. " + warn_message
                        )
                    rospy.logwarn(warn_message)
                    try:
                        clone_repo(
                            dep_clone_path,
                            dep_index_info["git_url"],
                            branch=(
                                dep_index_info["git_branch"]
                                if "git_branch" in dep_index_info.keys()
                                else "main"
                            ),
                            recursive=True,
                        )
                        deps_cloned = True
                    except Exception:
                        warn_msg = f"ROS dependency '{dep}' could not be installed."
                        rospy.logwarn(warn_msg)
                        return False
                else:
                    # Throw warning if dependency is installed but not up to date.
                    if outdated_warning and not is_repo_up_to_date(dep_clone_path):
                        rospy.logwarn(
                            f"Package dependency '{dep}' is installed but not up to "
                            "date. If you want to have the latest version of the "
                            "package, please update it manually by running 'git pull' "
                            f"in the package directory (i.e. '{dep_clone_path}')."
                        )
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
                f"Although package '{package_name}' was installed in the {space_str}, "
                "no information about it was found in the 'ros_gazebo_gym' dependency "
                f"index (i.e. '{rosdep_index_path}'). As a result, we could not "
                "determine whether its dependencies were installed. Please add this "
                "package to the index if you want to be sure all dependencies are "
                "installed."
            )
        else:
            rospy.logwarn(
                f"Package '{package_name}' is not installed or present in the "
                f"'ros_gazebo_gym' dependency index (i.e. '{rosdep_index_path}'). As a "
                "result, it could not be installed. Please install the package in the "
                "global or local ROS workspace or add its information to the "
                "dependency index."
            )
            return False

    # (Re)-build the catkin workspace.
    if deps_cloned:
        rospy.logwarn(
            "Installing system dependencies and (re)-building catkin workspace "
            f"'{workspace_path}' since new packages were added."
        )
        try:
            build_catkin_ws(workspace_path, install_ros_deps=True)
        except Exception:
            rospy.logerr(
                "While installing the package system dependencies and (re)-building "
                "the Catkin workspace, something went wrong. Please install the "
                "system dependencies, manually build the Catkin workspace, and try "
                "again."
            )
            ros_exit_gracefully(
                shutdown_msg=f"Shutting down {rospy.get_name()}", exit_code=1
            )

    return True


def load_ros_params_from_yaml(yaml_file_path, ros_package_name=None):
    """Loads ros parameters from yaml file.

    Args:
        yaml_file_path (str): The configuration file path. Can be absolute or relative.
            If relative, the path is relative to the package directory.
        package_name (str): The package name that contains the configuration file.
            Defaults to ``None``.

    Raises:
        :obj:`rosparam.RosParamException`: If the yaml file could not be loaded or the
            ROS package could not be found.
    """
    yaml_file_path = Path(yaml_file_path)

    # Look for the yaml file in the package directory.
    if ros_package_name is not None and not yaml_file_path.is_absolute():
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(ros_package_name)
        yaml_file_path = os.path.join(pkg_path, yaml_file_path)

    # Load parameters from yaml file.
    paramlist = rosparam.load_file(str(yaml_file_path))
    for params, ns in paramlist:
        params = remove_dict_none_values(params)
        rosparam.upload_params(ns, params)


def get_log_path():
    """Returns the 'ros_gazebo_gym' package log folder path.

    Returns:
        :obj:`pathlib.Path`: The package log folder path.
    """
    return Path(__file__).joinpath("../../../../logs").resolve()
