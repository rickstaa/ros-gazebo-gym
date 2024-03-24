"""Contains a class that can be used for displaying a marker for the cube object in
RViz.

.. note::
    This class overloads the :class:`ros_gazebo_gym.common.markers.target_marker.TargetMarker`
    class in order to pre-initialize some of its attributes. Most importantly, a pose
    offset was applied to align the marker frame with the object frame.
"""  # noqa: E501

from geometry_msgs.msg import Vector3
from ros_gazebo_gym.common.markers import TargetMarker
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class CubeMarker(TargetMarker):
    """RViz grasping cube marker.

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
        """Initialize CubeMarker object.

        Args:
            *args: Arguments passed to the
                :class:`~ros_gazebo_gym.common.markers.target_marker.TargetMarker` super
                class.
            **kwargs: Keyword arguments that are passed to the
                :class:`~ros_gazebo_gym.common.markers.target_marker.TargetMarker` super
                class.
        """
        super().__init__(*args, **kwargs)

        # Overwrite attributes with defaults if not supplied in the constructor.
        if "color" not in kwargs.keys():
            self.color = ColorRGBA(0.0, 0.0, 0.0, 1.0)
        if "scale" not in kwargs.keys():
            self.scale = Vector3(0.06, 0.06, 0.06)
        self.type = Marker.CUBE if "type" not in kwargs.keys() else self.type

        # Apply offset to align marker frame with cube object frame.
        self.pose.position.x += 0.03
        self.pose.position.y += 0.03
        self.pose.position.z += 0.03
