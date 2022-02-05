#!/usr/bin/env python3
"""Launches all the ROS nodes that are needed for a given
:ros_gazebo_gym:`ros_gazebo_gym <>` gym environment.
"""
import atexit
import subprocess
import sys
import psutil
from pathlib import Path
import rosgraph
import socket
import os

import catkin
import rospy
from ros_gazebo_gym.core.helpers import get_global_pkg_path, package_installer
from ros_gazebo_gym.common.functions import colorize


class ROSLauncher(object):
    """Class used to launch ROS launch files.

    Attributes:
        successful (bool): Whether the launch file was successfully launched. This only
            specifies if the launchfile was successfully executed not if it encountered
            errors.
    """

    launched = {}  # Stores all processes that were launched

    @classmethod
    def initialize(cls):
        """Make sure a ros master is running and ROS is
        initialized.
        """
        # Make sure ROS master is running
        try:
            if not rosgraph.is_master_online():
                print(
                    colorize(
                        "WARNING: No ROS master was found. Starting one in a "
                        "subprocess.",
                        color="yellow",
                        bold=True,
                    )
                )
                p = subprocess.Popen("roscore")
                state = p.poll()
                if state is None:
                    print(colorize("INFO: ROS master successfully started."))
                    cls.launched["roscore"] = p  # Store a reference to the process
                    atexit.register(
                        cls.kill_process, process_name="roscore"
                    )  # Make sure the roscore dies when parent dies
                elif state != 0:
                    print(
                        colorize(
                            "ERROR: Something went wrong while trying to launch the "
                            "ROS master.",
                            color="red",
                            bold=True,
                        )
                    )
                    sys.exit(0)
        except socket.error:
            print(
                colorize(
                    "ERROR: No ROS master was found and none could be started "
                    "please check your system and try again.",
                    color="red",
                    bold=True,
                )
            )
            sys.exit(0)

        # Make sure init_node has been called
        if not rospy.rostime.is_rostime_initialized():
            rospy.init_node(
                "ros_gazebo_gym_launcher_node", anonymous=True, log_level=rospy.WARN
            )

    @classmethod
    def kill_process(cls, process_name):
        """Kill a running process and its child processes.

        Args:
            process_name (str): The process name.
        """
        process = psutil.Process(cls.launched[process_name].pid)
        for proc in process.children(recursive=True):
            proc.kill()
        process.kill()

    @classmethod
    def launch(
        cls,
        package_name,
        launch_file_name,
        workspace_path=None,
        log_file=None,
        **kwargs,
    ):
        """Launch a given launchfile while also installing the launchfile package and or
        dependencies. This is done by using the ros_gazebo_gym dependency index.

        Args:
            package_name (str): The package that contains the launchfile.
            launch_file_name (str): The launchfile name.
            workspace_path (str, optional): The path of the catkin workspace. Defaults
                to ``None`` meaning the path will be determined.
            log_file(str, optional): The log file to write the ``stdout`` to. Defaults
                to ``None`` meaning the ``stdout`` will be written to console.
            **kwargs: Keyword arguments you want to pass to the launchfile.

        Raises:
            Exception: When something went wrong when launching the launchfile.
        """
        # Retrieve workspace path
        workspace_path = (
            workspace_path
            if workspace_path
            else catkin.workspace.get_workspaces()[0].replace("/devel", "")
        )
        if not workspace_path:
            rospy.logerr(
                "Workspace path could not be found. Please make sure that you source "
                "the workspace before calling the ROSLauncher or supply it with a "
                "workspace_path."
            )
            sys.exit(0)

        # Install launch file dependencies if they are not present
        try:
            package_installed = package_installer(
                package_name, workspace_path=workspace_path
            )
        except Exception:
            rospy.logwarn(
                f"Something went wrong while trying to install the '{package_name}' "
                "ROS package and its dependencies."
            )
            package_installed = False

        # Launch launch file if package was found
        if package_installed:
            rospy.loginfo(
                f"Starting '{launch_file_name}' launch file from package "
                f"'{package_name}."
            )
            pkg_name = get_global_pkg_path(package_name)
            if pkg_name:
                launch_dir = Path(get_global_pkg_path(package_name)).joinpath("launch")
                path_launch_file_name = Path(launch_dir).joinpath(launch_file_name)
                rospy.logdebug(f"Launch file path: {path_launch_file_name}")
            # NOTE: Bash prefix needed since sourcing setup.sh doesn't seem to work
            bash_prefix = '/bin/bash -c "'
            source_command = ". {}{};".format(
                workspace_path, Path("/devel/setup.bash").resolve()
            )
            roslaunch_command = "roslaunch {} {}".format(package_name, launch_file_name)
            kwargs_command = (
                " " + " ".join([f"{key}:={val}" for key, val in kwargs.items()])
                if kwargs
                else ""
            )
            command = (
                bash_prefix + source_command + roslaunch_command + kwargs_command + '"'
            )
            rospy.logwarn("Launching command: " + str(command))

            # Launch the launchfile using a subprocess.
            # NOTE: I also tried using the roslaunch python api but I could not find a
            # way to first source the catkin workspace.
            if log_file is not None:
                os.makedirs(Path(log_file).parent, exist_ok=True)
                log_file = open(log_file, "w")
            p = subprocess.Popen(
                command, shell=True, cwd=workspace_path, stdout=log_file, stderr=log_file
            )
            state = p.poll()
            if state is None:
                rospy.logdebug("Launch file successfully launched.")
                cls.launched[launch_file_name] = p  # Store a reference to the process
                atexit.register(
                    cls.kill_process, process_name=launch_file_name
                )  # Make sure process dies when parent dies
            elif state != 0:
                rospy.logerror(
                    "Something went wrong while trying to launch the "
                    f"{path_launch_file_name} launch file."
                )
        else:
            raise Exception(f"Package '{package_name}' could not be found.")
