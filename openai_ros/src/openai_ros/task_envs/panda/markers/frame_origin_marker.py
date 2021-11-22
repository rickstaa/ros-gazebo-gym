"""Class used for displaying a marker for the object frame origin in rviz. This class
overloads the :obj:`openai_ros.common.markers.TargetMarker` class in order to
pre-initialize some of its attributes.
"""

from openai_ros.common.markers import TargetMarker
from std_msgs.msg import ColorRGBA


class FrameOriginMarker(TargetMarker):
    """Class used to create an rviz marker for the object frame origin.

    Attributes:
        x (int): The marker x position.
        y (int): The marker y position.
        z (int): The marker z position.
        id (int): The marker object id.
        type (str): The marker type.
        action (float): The marker message action (add or remove).
        pose (:obj:`geometry_msgs.Pose`): The marker pose.
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

    def __init__(self, *args, **kwds):
        """Initialize FrameOriginMarker object.

        Args:
            *args: Arguments passed to the
                :obj:`~openai_ros.common.markers.TargetMarker` super class.
            **kwargs: Keyword arguments that are passed to the
                :obj:`~openai_ros.common.markers.TargetMarker` super class.
        """
        super().__init__(*args, **kwds)

        # Overwrite attributes with defaults if not supplied in the constructor
        if "color" not in kwds.keys():
            self.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)