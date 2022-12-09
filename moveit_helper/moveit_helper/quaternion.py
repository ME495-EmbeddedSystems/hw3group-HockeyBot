"""Convert from angle-axis to quaternion and calculate the distance between 2 points."""
from math import sin, cos


def euler_quaternion(r, p, y):
    """
    Convert from roll, pitch, yaw of rotation to a quaternion.

    Args:
    ----
      r: The roll (rotation around x-axis) angle in radians
      p: The pitch (rotation around y-axis) angle in radians
      y: The yaw (rotation around z-axis) angle in radians

    Return:
    ------
      A Quaternion corresponding to the rotation

    """
    x = sin(r/2) * cos(p/2) * cos(y/2) - cos(r/2) * sin(p/2) * sin(y/2)
    y = cos(r/2) * sin(p/2) * cos(y/2) + sin(r/2) * cos(p/2) * sin(y/2)
    z = cos(r/2) * cos(p/2) * sin(y/2) - sin(r/2) * sin(p/2) * cos(y/2)
    w = cos(r/2) * cos(p/2) * cos(y/2) + sin(r/2) * sin(p/2) * sin(y/2)

    return x, y, z, w
