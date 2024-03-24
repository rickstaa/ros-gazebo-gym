"""Contains a class that can be used for displaying text in RViz.

.. note::
    This class overloads the
    :obj:`jsk_rviz_plugins.msgs.OverlayText` class in order to pre-initialize some of
    its attributes. It further automatically makes sure the text width and height are
    not to small.
"""

import math

from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA


class TextOverlay(OverlayText):
    """RViz text overlay.

    .. note::
        When no width or height are supplied it makes sure that the display fits all the
        text.

    Attributes:
        action (float): The marker message action (add or remove).
        width (int): The overlay width.
        height (int): The overlay height.
        left (int): Position relative to the left.
        top (int): Position relative to the top.
        bg_color (:obj:`std_msgs.ColorRGBA`): The overlay background color.
        fg_color (:obj:`std_msgs.ColorRGBA`): The text color.
        line_width (int): The text line width.
        font (str): The font.
        text (str): The text.
    """

    def __init__(self, *args, **kwargs):
        """Initialize text overlay object."""
        self.__text_size = (
            0.0 if "text_size" not in kwargs.keys() else kwargs["text_size"]
        )
        self._text = "" if "text" not in kwargs.keys() else kwargs["text"]
        super().__init__(*args, **kwargs)
        self._fit_width_to_text()
        self._fit_height_to_text()

        # Overwrite attributes with defaults if not supplied in the constructor.
        if "bg_color" not in kwargs.keys():
            self.bg_color = ColorRGBA(0.0, 0.0, 0.0, 1.0)
        if "fg_color" not in kwargs.keys():
            self.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        if "left" not in kwargs.keys():
            self.left = 10
        if "top" not in kwargs.keys():
            self.top = 10
        if "width" in kwargs.keys():
            self.width = kwargs["width"]
        if "height" in kwargs.keys():
            self.height = kwargs["height"]
        self.action = OverlayText.ADD if "action" not in kwargs.keys() else self.action

    def _fit_width_to_text(self):
        """Makes sure that the width is big enough to hold the text."""
        text_size = self.__text_size if self.__text_size != 0.0 else 11
        self.width = math.ceil(
            max([len(line) for line in self._text.split("\n")]) * text_size * 0.731
        )

    def _fit_height_to_text(self):
        """Makes sure that the height is high enough to hold the text."""
        text_size = self.__text_size if self.__text_size != 0.0 else 11
        lines = len(self._text.split("\n"))
        self.height = math.ceil(lines * (text_size * 3 / 2))

    @property
    def text_size(self):
        """Retrieve the text size."""
        return self.__text_size

    @text_size.setter
    def text_size(self, val):
        """Set text size while making sure the width and height are correct."""
        if val:
            self.__text_size = val
            self._fit_width_to_text()
            self._fit_height_to_text()
