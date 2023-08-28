"""Contains a class used for storing euler angles.

.. note::
    Uses the ``ypr`` (x-y-z) euler angle convention.
"""


class EulerAngles(object):
    """Used for storing euler angles.

    Attributes:
        y (float): Yaw angle (z). Defaults to ``0.0``.
        p (float): Pitch angle (p). Defaults to ``0.0``.
        r (float): Roll angle (r). Defaults to ``0.0``.
    """

    def __init__(self, y=0.0, p=0.0, r=0.0):
        """Initializes the EulerAngles object.

        Args:
            y (float, optional): Yaw angle (z). Defaults to ``0.0``.
            p (float, optional): Pitch angle (p). Defaults to ``0.0``.
            r (float, optional): Roll angle (r). Defaults to ``0.0``.
        """
        self.y = y
        self.p = p
        self.r = r
