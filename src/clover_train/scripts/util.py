import math
import os
import subprocess
import xml.etree.ElementTree as ET
import random
import copy

from typing import List, Any, Tuple, Dict, TypeVar
from dataclasses import fields, asdict
from geometry_msgs.msg import Pose

T = TypeVar("T")


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


def save_clover_train_metadata(metadata: Dict[str, List], identifier: str) -> None:
    """
    Save training run metadata in a file unique to the current time.

    Any kind of `metadata` can be provided; the columns will be dynamically named in the file.

    file_id: identifier to make the file unique, e.g., the current date and time in ISO format
    """
    clover_train_metadata_dir = os.path.join(
        os.path.expanduser("~"), "clover_train_metadata"
    )
    os.makedirs(clover_train_metadata_dir, exist_ok=True)
    clover_train_metadata_data_path = os.path.join(
        clover_train_metadata_dir, f"clover_train-{identifier}.txt"
    )
    with open(clover_train_metadata_data_path, "a+") as file:
        # write headers
        file.write(" ".join([key for key in metadata]))
        file.write("\n")
        # convert list of columns into list of rows
        metadata_rows = [list(row_tuple) for row_tuple in zip(*metadata.values())]
        for row in metadata_rows:
            file.write(" ".join([str(i) for i in row]))
            file.write("\n")


def rosclean_purge() -> None:
    """Purge ROS log files."""
    subprocess.run(["rosclean", "purge", "-y"])


def parse_world_free_poses_xml(filepath: str) -> List[Pose]:
    """Parse calculated free poses from xml format into a list of poses."""
    tree = ET.parse(filepath)
    root = tree.getroot()
    res = []
    for pose_element in root:
        pose_element_obj = Pose()
        pose_element_obj.position.x = float(pose_element.find("x").text)
        pose_element_obj.position.y = float(pose_element.find("y").text)
        res.append(pose_element_obj)
    return res
