#!/usr/bin/env python3

import os

import roslaunch
import rospy
import rospkg


def launch_simulation():
    """Launch all nodes."""
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    rospack = rospkg.RosPack()
    clover_simulation_path = rospack.get_path("clover_simulation")
    this_launch = roslaunch.parent.ROSLaunchParent(
        uuid, [os.path.join(clover_simulation_path, "launch", "simulator.launch")]
    )
    this_launch.start()
