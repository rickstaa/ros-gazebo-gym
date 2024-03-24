"""Contains a class that can be used for displaying a sampling region marker in RViz.

.. note::
    This class overloads the :obj:`visualization_msgs.msgs.Marker` class in order to
    pre-initialize some of its attributes. It further also adds the ability to specify
    the marker scale using s``x``, ``y``, ``z`` max and min values.
"""

import rospy
from geometry_msgs.msg import Pose, Vector3
from ros_gazebo_gym.common.helpers import normalize_quaternion
from rospy.exceptions import ROSInitException
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker


class SampleRegionMarker(Marker):
    """RViz goal sample region marker.

    Attributes:
        id (int): The marker object id.
        type (str): The marker type.
        action (float): The marker message action (add or remove).
        color (:obj:`std_msgs.ColorRGBA`): The marker color.
        lifetime (:obj:`:rospy.Duration`): The lifetime duration.
        frame_locked (bool): Boolean specifying whether the marker frame is locked to
            the world.
        point (:obj:`geometry_msgs.Point`): The marker points.
        text (str): The text that is used for text markers.
        mesh_resource (str): Marker mess location.
        mesh_use_embedded_materials (bool): Boolean specifying whether we want to use a
            mesh.

    .. important::
        The x, y, z min/max values take precedence over x y z position values, which
        take precedence over position and scale values.
    """

    def __init__(  # noqa: C901
        self,
        x=0,
        y=0,
        z=0,
        x_min=None,
        y_min=None,
        z_min=None,
        x_max=None,
        y_max=None,
        z_max=None,
        frame_id=None,
        *args,
        **kwargs
    ):
        """Initialize SampleRegionMarker object.

        Args:
            x_min (float, optional): The min x position of the marker. Defaults to
                ``None``.
            y_min (float, optional): The min y position of the marker. Defaults to
                ``None``.
            z_min (float, optional): The min z position of the marker. Defaults to
                ``None``.
            x_max (float, optional): The max x position of the marker. Defaults to
                ``None``.
            y_max (float, optional): The max y position of the marker. Defaults to
                ``None``.
            z_max (float, optional): The max z position of the marker. Defaults to
                ``None``.
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
        self.__scale = Vector3()
        self.__x_min = 0.0
        self.__x_max = 0.0
        self.__y_min = 0.0
        self.__z_min = 0.0
        self.__y_max = 0.0
        self.__z_max = 0.0
        super().__init__(*args, **kwargs)

        # Overwrite attributes with defaults if not supplied in the constructor.
        if "header" not in kwargs.keys():
            # Pre-initialize header.
            self.header = Header()
            try:  # Check if rostime was initialized.
                self.header.stamp = rospy.Time.now()
            except ROSInitException:
                raise Exception(
                    "Goal sample region marker could not be created as the ROS time is "
                    "not initialized. Have you called init_node()?"
                )
            if not frame_id:
                self.header.frame_id = "world"
            else:
                self.header.frame_id = frame_id
        if "pose" not in kwargs.keys():
            self.pose = Pose(position=Vector3(x, y, z))
        if "color" not in kwargs.keys():
            self.color = ColorRGBA(1.0, 0.0, 0.0, 0.15)
        self.id = 1 if "id" not in kwargs.keys() else self.id
        self.type = Marker.CUBE if "type" not in kwargs.keys() else self.type
        self.action = Marker.ADD if "action" not in kwargs.keys() else self.action
        self.lifetime = (
            rospy.Duration(0) if "lifetime" not in kwargs.keys() else self.lifetime
        )

        # Set class properties if not none.
        if x:
            self.x = x
        if y:
            self.y = y
        if z:
            self.z = z
        if x_min:
            self.x_min = x_min
        if x_max:
            self.x_max = x_max
        if y_min:
            self.y_min = y_min
        if y_max:
            self.y_max = y_max
        if z_min:
            self.z_min = z_min
        if z_max:
            self.z_max = z_max

    @property
    def x(self):
        """Retrieve the marker x position."""
        return self.__x

    @x.setter
    def x(self, val):
        """Set the marker x position."""
        self.__x = val
        self.__pose.position.x = val
        self.__x_min = self.__pose.position.x - self.__scale.x / 2
        self.__x_max = self.__pose.position.x + self.__scale.x / 2

    @property
    def y(self):
        """Retrieve the marker y position."""
        return self.__y

    @y.setter
    def y(self, val):
        """Set the marker y position."""
        self.__y = val
        self.__pose.position.y = val
        self.__y_min = self.__pose.position.y - self.__scale.y / 2
        self.__y_max = self.__pose.position.y + self.__scale.y / 2

    @property
    def z(self):
        """Retrieve the marker z position."""
        return self.__z

    @z.setter
    def z(self, val):
        """Set the marker z position."""
        self.__z = val
        self.__pose.position.z = val
        self.__z_min = self.__pose.position.z - self.__scale.z / 2
        self.__z_max = self.__pose.position.z + self.__scale.z / 2

    @property
    def x_min(self):
        """Retrieve the value of x_min."""
        return self.__x_min

    @x_min.setter
    def x_min(self, val):
        """Set the value of x_min and update the x scale and position."""
        self.__x_min = val
        self.__scale.x = abs(self.__x_max - self.__x_min)
        self.__pose.position.x = self.__scale.x / 2 + self.__x_min

    @property
    def y_min(self):
        """Retrieve the value of y_min."""
        return self.__y_min

    @y_min.setter
    def y_min(self, val):
        """Set the value of y_min and update the y scale and position."""
        self.__y_min = val
        self.__scale.y = abs(self.__y_max - self.__y_min)
        self.__pose.position.y = self.__scale.y / 2 + self.__y_min

    @property
    def z_min(self):
        """Retrieve the value of z_min."""
        return self.__z_min

    @z_min.setter
    def z_min(self, val):
        """Set the value of z_min and update the z scale and position."""
        self.__z_min = val
        self.__scale.z = abs(self.__z_max - self.__z_min)
        self.__pose.position.z = self.__scale.z / 2 + self.__z_min

    @property
    def x_max(self):
        """Retrieve the value of x_max."""
        return self.__x_max

    @x_max.setter
    def x_max(self, val):
        """Set the value of x_max and update the x scale and position."""
        self.__x_max = val
        self.__scale.x = abs(self.__x_max - self.__x_min)
        self.__pose.position.x = self.__scale.x / 2 + self.__x_min

    @property
    def y_max(self):
        """Retrieve the value of y_max."""
        return self.__y_max

    @y_max.setter
    def y_max(self, val):
        """Set the value of y_max and update the y scale and position."""
        self.__y_max = val
        self.__scale.y = abs(self.__y_max - self.__y_min)
        self.__pose.position.y = self.__scale.y / 2 + self.__y_min

    @property
    def z_max(self):
        """Retrieve the value of z_max."""
        return self.__z_max

    @z_max.setter
    def z_max(self, val):
        """Set the value of z_max and update the z scale and position."""
        self.__z_max = val
        self.__scale.z = abs(self.__z_max - self.__z_min)
        self.__pose.position.z = self.__scale.z / 2 + self.__z_min

    @property
    def scale(self):
        """Get marker scale."""
        return self.__scale

    @scale.setter
    def scale(self, val):
        """Set marker scale."""
        if val:
            self.__scale = val
            self.__x_min = self.__pose.position.x - self.__scale.x / 2
            self.__x_max = self.__pose.position.x + self.__scale.x / 2
            self.__y_min = self.__pose.position.y - self.__scale.y / 2
            self.__y_max = self.__pose.position.y + self.__scale.y / 2
            self.__z_min = self.__pose.position.z - self.__scale.z / 2
            self.__z_max = self.__pose.position.z + self.__scale.z / 2

    @property
    def pose(self):
        """Retrieve marker pose."""
        return self.__pose

    @pose.setter
    def pose(self, val):
        """Set marker pose."""
        if val:
            self.__pose = Pose(
                position=val.position,
                orientation=normalize_quaternion(val.orientation),
            )
            self.__x = self.__pose.position.x
            self.__y = self.__pose.position.y
            self.__z = self.__pose.position.z
            self.__x_min = self.__pose.position.x - self.__scale.x / 2
            self.__x_max = self.__pose.position.x + self.__scale.x / 2
            self.__y_min = self.__pose.position.y + self.__scale.y / 2
            self.__y_max = self.__pose.position.y - self.__scale.y / 2
            self.__z_min = self.__pose.position.z - self.__scale.z / 2
            self.__z_max = self.__pose.position.z + self.__scale.z / 2
