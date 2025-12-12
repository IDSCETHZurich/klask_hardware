"""Utility functions"""

from geometry_msgs.msg import Polygon, Point32


def create_point32(self, x: float, y: float, z: float = 0.0) -> Point32:
    """Create a Point32 message from coordinates."""
    point = Point32()
    point.x = float(x)
    point.y = float(y)
    point.z = z
    return point
