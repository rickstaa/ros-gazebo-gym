﻿"""Contains a class that can be used for displaying a target marker in RViz.

.. note::
    This class overloads the :obj:`visualization_msgs.msgs.Marker` class in order to
    pre-initialize some of its attributes.
"""
import rospy
import tf2_ros
from geometry_msgs.msg import Pose, Transform, TransformStamped, Vector3
from ros_gazebo_gym.common.helpers import normalize_quaternion
from rospy.exceptions import ROSInitException
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker


class TargetMarker(Marker):
    """RViz target goal marker.

    Attributes:
        id (int): The marker object id.
        type (str): The marker type.
        action (float): The marker message action (add or remove).
        scale (:obj:`geometry_msgs.Vector3`): The marker scale.
        color (:obj:`std_msgs.ColorRGBA`): The marker color.
        lifetime (:obj:`rospy.duration`): The lifetime duration.
        frame_locked (bool): Boolean specifying whether the marker frame is locked to
            the world.
        point (:obj:`geometry_msgs.Point`): The marker points.
        text (str): The text that is used for text markers.
        mesh_resource (str): Marker mess location.
        mesh_use_embedded_materials (bool): Boolean specifying whether we want to use a
            mesh.

    .. important::
        If both the x,y,z positions and a Pose is supplied the x,y,z positions are used.
    """

    def __init__(  # noqa: C901
        self,
        x=0,
        y=0,
        z=0,
        frame_id=None,
        *args,
        **kwargs,
    ):
        """Initialize TargetMarker object.

        Args:
            x (int, optional): The target marker x position. Defaults to ``0``.
            y (int, optional): The target marker y position. Defaults to ``0``.
            z (int, optional): The target marker z position. Defaults to ``0``.
            frame_id (str, optional): The frame the position/orientation are relative
                to. Defaults to ``None``.
            *args: Arguments passed to the
                :obj:`~visualization_msgs.msg.Marker` super class.
            **kwargs: Keyword arguments that are passed to the
                :obj:`~visualization_msgs.msg.Marker` super class.

        Raises:
            Exception: Thrown when the ROS time is not initialised.
        """
        self.__pose = Pose()
        super().__init__(*args, **kwargs)

        # Overwrite attributes with defaults if not supplied in the constructor.
        if "header" not in kwargs.keys():
            # Pre-initialize header.
            self.header = Header()
            try:  # Check if rostime was initialized.
                self.header.stamp = rospy.Time.now()
            except ROSInitException:
                raise Exception(
                    "Marker could not be created as the ROS time is not "
                    "initialized. Have you called init_node()?"
                )
            if not frame_id:
                self.header.frame_id = "world"
            else:
                self.header.frame_id = frame_id
        if "pose" not in kwargs.keys():
            self.pose = Pose(position=Vector3(x, y, z))
        if "color" not in kwargs.keys():
            self.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        if "scale" not in kwargs.keys():
            self.scale = Vector3(0.025, 0.025, 0.025)
        self.id = 0 if "id" not in kwargs.keys() else self.id
        self.type = Marker.SPHERE if "type" not in kwargs.keys() else self.type
        self.action = Marker.ADD if "action" not in kwargs.keys() else self.action
        self.lifetime = (
            rospy.Duration(0) if "lifetime" not in kwargs.keys() else self.lifetime
        )

        # Set marker x/y/z positions if not none.
        if x:
            self.x = x
        if y:
            self.y = y
        if z:
            self.z = z

        # Create TF broadcaster.
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

    def publish_tf(self, tf_frame_name):
        """Publish a tf frame for the marker.

        Args:
            tf_frame_name (str, optional): The name used for the tf frame. Defaults to
                ``None`` (i.e. frame name will be autogenerated).
        """
        child_frame_id = tf_frame_name or f"target_marker_{self.id}"
        tf_msg = TransformStamped(
            header=Header(frame_id=self.header.frame_id, stamp=rospy.Time.now()),
            child_frame_id=child_frame_id,
            transform=Transform(
                translation=self.pose.position,
                rotation=self.pose.orientation,
            ),
        )
        self._tf_broadcaster.sendTransform(tf_msg)

    @property
    def x(self):
        """Retrieve the marker x position."""
        return self.__x

    @x.setter
    def x(self, val):
        """Set the marker x value."""
        if val:
            self.__x = val
            self.__pose.position.x = val

    @property
    def y(self):
        """Retrieve the marker y position."""
        return self.__y

    @y.setter
    def y(self, val):
        """Set the marker y value."""
        if val:
            self.__y = val
            self.__pose.position.y = val

    @property
    def z(self):
        """Retrieve the marker z position."""
        return self.__z

    @z.setter
    def z(self, val):
        """Set the marker y value."""
        if val:
            self.__z = val
            self.__pose.position.z = val

    @property
    def pose(self):
        """Retrieve the marker pose."""
        return self.__pose

    @pose.setter
    def pose(self, val):
        """Set the marker pose.

        Args:
            val (:obj:`geometry_msgs.msg.Pose): The marker pose.
        """
        if val:
            self.__pose = Pose(
                position=val.position,
                orientation=normalize_quaternion(val.orientation),
            )
            self.__x = val.position.x
            self.__y = val.position.y
            self.__z = val.position.z
