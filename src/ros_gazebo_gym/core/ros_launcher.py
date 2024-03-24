"""Launches all the ROS nodes that are needed for a given
:ros-gazebo-gym:`ros_gazebo_gym <>` gymnasium environment.
"""

import os
import socket
import time
from pathlib import Path

import rosgraph
import rospy
from ros_gazebo_gym.core.helpers import (
    PopenAutoCleanup,
    get_catkin_workspace_path,
    get_global_pkg_path,
    install_package,
    ros_exit_gracefully,
)


class ROSLauncher(object):
    """Class used to launch ROS launch files.

    Attributes:
        successful (bool): Whether the launch file was successfully launched. *This
            only specifies if the launchfile was successfully executed not if it
            encountered errors*.
    """

    launched = {}  # Stores all processes that were launched.

    @classmethod
    def initialize(cls):
        """Make sure a ros master is running and ROS is initialized."""
        try:
            # Ensure that a ROS master is running.
            if not rosgraph.is_master_online():
                rospy.logwarn("No ROS master was found. Starting one in a subprocess.")
                p = PopenAutoCleanup("roscore", critical=True)
                state = p.poll()
                if state is None:
                    rospy.loginfo("ROS master successfully started.")
                    cls.launched["roscore"] = p  # Store a reference to the process.
                elif state != 0:
                    rospy.logerr(
                        "Something went wrong while trying to launch the ROS master."
                    )
                    ros_exit_gracefully(
                        shutdown_msg=f"Shutting down {rospy.get_name()}", exit_code=1
                    )
        except socket.error:
            rospy.logerr(
                "No ROS master was found and none could be started please check your "
                "system and try again."
            )
            ros_exit_gracefully(
                shutdown_msg=f"Shutting down {rospy.get_name()}", exit_code=1
            )

        # Make sure ROS has been initialized.
        if not rospy.rostime.is_rostime_initialized():
            rospy.init_node("ros_gazebo_gym_launcher_node", anonymous=True)

    @classmethod
    def list_processes(cls):
        """List all launched processes."""
        for process_name in cls.launched:
            print(f"{process_name}: {cls.launched[process_name].pid}")

    @classmethod
    def terminate(cls, process_name):
        """Terminate a launched process.

        Args:
            process_name (str): The process name.
        """
        if process_name in cls.launched:
            cls.launched[process_name].terminate()
            del cls.launched[process_name]

    @classmethod
    def terminate_all(cls):
        """Terminate all launched processes."""
        for process_name in cls.launched:
            cls.launched[process_name].terminate()
        cls.launched = {}

    @classmethod
    def launch(  # noqa: C901
        cls,
        package_name,
        launch_file_name,
        workspace_path=None,
        log_file=None,
        critical=False,
        wait_time=2,
        outdated_warning=False,
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
            critical (bool, optional): Whether the process is critical and an error
                message should be shown when the process is no longer running.
            wait_time (int, optional): The time to wait before checking if the was
                launched successfully and is still running. Defaults to ``2``.
            outdated_warning (bool, optional): Whether to show a update warning when the
                package is outdated. Defaults to ``False``.
            **kwargs: Keyword arguments you want to pass to the launchfile.

        Raises:
            Exception: When something went wrong when launching the launchfile.
        """
        if not workspace_path:
            workspace_path = get_catkin_workspace_path()
        if not workspace_path:
            rospy.logerr(
                "Workspace path could not be found. Please make sure that you source "
                "the workspace before calling the ROSLauncher or supply it with a "
                "workspace_path."
            )
            ros_exit_gracefully(
                shutdown_msg=f"Shutting down {rospy.get_name()}", exit_code=1
            )

        # Install ROS package and its dependencies if they are not present.
        try:
            package_installed = install_package(
                package_name,
                workspace_path=workspace_path,
                outdated_warning=outdated_warning,
            )
        except Exception:
            rospy.logwarn(
                f"Something went wrong while trying to install the '{package_name}' "
                "ROS package and its dependencies."
            )
            package_installed = False

        # Launch launch file if package was found.
        if package_installed:
            # NOTE: Bash prefix needed since sourcing 'setup.sh' doesn't seem to work.
            rospy.loginfo(
                f"Starting '{launch_file_name}' launch file from package "
                f"'{package_name}."
            )
            pkg_name = get_global_pkg_path(package_name)
            if pkg_name:
                launch_dir = Path(get_global_pkg_path(package_name)).joinpath("launch")
                path_launch_file_name = Path(launch_dir).joinpath(launch_file_name)
                rospy.logdebug(f"Launch file path: {path_launch_file_name}")
            source_command = ". ./devel/setup.bash;"
            roslaunch_command = "roslaunch {} {}".format(package_name, launch_file_name)
            kwargs_command = (
                " " + " ".join([f"{key}:={val}" for key, val in kwargs.items()])
                if kwargs
                else ""
            )
            command = source_command + roslaunch_command + kwargs_command

            rospy.logwarn("Launching command: " + str(command))

            # Launch the launchfile using a subprocess.
            # NOTE: Subprocess used instead of the roslaunch python api since it does
            # not provide a way to first source the catkin workspace.
            if log_file is not None:
                os.makedirs(Path(log_file).parent, exist_ok=True)
                log_file = open(log_file, "w")
            p = PopenAutoCleanup(
                command,
                critical=critical,
                executable="/bin/bash",
                shell=True,
                cwd=workspace_path,
                stdout=log_file,
                stderr=log_file,
            )
            time.sleep(wait_time)  # Wait a bit to make sure the process is running.
            state = p.poll()
            if state is None:
                rospy.logdebug("Launch file successfully launched.")
                cls.launched[launch_file_name] = p  # Store a reference to the process.
            else:
                rospy.logerr(
                    "Something went wrong while trying to launch the "
                    f"{path_launch_file_name} launch file."
                )
                ros_exit_gracefully(
                    shutdown_msg=f"Shutting down {rospy.get_name()}", exit_code=1
                )
        else:
            raise Exception(
                f"Package '{package_name}' and its dependencies could not be found."
            )
