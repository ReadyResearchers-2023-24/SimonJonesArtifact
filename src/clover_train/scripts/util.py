import math

from typing import List, Any, Tuple
from dataclasses import fields, asdict


def convert_spherical_to_cartesian(
    r: float, theta: float, phi: float
) -> Tuple[float, float, float]:
    """Convert spherical coordinates to cartesian coordinates."""
    x: float = r * math.sin(theta) * math.cos(phi)
    y: float = r * math.sin(theta) * math.sin(phi)
    z: float = r * math.cos(theta)
    print(f"(x, y, z): {x} {y} {z}")
    return (x, y, z)


def quaternion_to_euler_angles(
    qx: float, qy: float, qz: float, qw: float
) -> Tuple[float, float, float]:
    """Convert quarternion form to euler angles."""
    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = math.sqrt(1 + 2 * (qw * qy - qx * qz))
    cosp = math.sqrt(1 - 2 * (qw * qy - qx * qz))
    pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


def count_dataclass_fields(the_dataclass) -> int:
    """Count the number of fields in a dataclass."""
    dummy_instance = the_dataclass()
    return len([field.name for field in fields(dummy_instance)])


def dataclass_to_list(the_dataclass_instance) -> List[Any]:
    """Convert a dataclass instance to a list."""
    return list(asdict(the_dataclass_instance).values())
