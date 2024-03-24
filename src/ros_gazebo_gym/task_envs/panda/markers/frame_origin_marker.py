"""Contains a class that can be used for displaying a marker for the object frame origin
in RViz.

.. note::
    This class overloads the :obj:`ros_gazebo_gym.common.markers.target_marker.TargetMarker`
    class in order to pre-initialize some of its attributes.
"""  # noqa: E501

from ros_gazebo_gym.common.markers import TargetMarker
from std_msgs.msg import ColorRGBA


class FrameOriginMarker(TargetMarker):
    """RViz object frame origin marker.

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
        If both the ``x``, ``y``, ``z`` positions and a Pose is supplied the
        ``x``, ``y``, ``z`` positions are used.
    """

    def __init__(self, *args, **kwargs):
        """Initialize FrameOriginMarker object.

        Args:
            *args: Arguments passed to the
                :obj:`~ros_gazebo_gym.common.markers.target_marker.TargetMarker` super
                class.
            **kwargs: Keyword arguments that are passed to the
                :obj:`~ros_gazebo_gym.common.markers.target_marker.TargetMarker` super
                class.
        """
        super().__init__(*args, **kwargs)

        # Overwrite attributes with defaults if not supplied in the constructor.
        if "color" not in kwargs.keys():
            self.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
