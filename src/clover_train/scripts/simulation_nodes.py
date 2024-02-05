#!/usr/bin/env python3

import os

import roslaunch
import rospy

px4_launch = None
gazebo_launch = None
clover_services_launch = None
clover_model_launch = None


def launch_px4():
    """Launch px4 node."""
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    px4_launch = roslaunch.parent.ROSLaunchParent(
        uuid, [os.path.join("launch", "px4.launch")]
    )
    px4_launch.start()
    rospy.loginfo("started px4")


def launch_gazebo():
    """Launch gazebo node."""
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    gazebo_launch = roslaunch.parent.ROSLaunchParent(
        uuid, [os.path.join("launch", "gazebo.launch")]
    )
    gazebo_launch.start()
    rospy.loginfo("started gazebo")


def launch_clover_services():
    """Launch clover_services node."""
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    clover_services_launch = roslaunch.parent.ROSLaunchParent(
        uuid, [os.path.join("launch", "clover_services.launch")]
    )
    clover_services_launch.start()
    rospy.loginfo("started clover_services")


def launch_clover_model():
    """Launch clover_model node."""
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    clover_model_launch = roslaunch.parent.ROSLaunchParent(
        uuid, [os.path.join("launch", "clover_model.launch")]
    )
    clover_model_launch.start()
    rospy.loginfo("started clover_model")
