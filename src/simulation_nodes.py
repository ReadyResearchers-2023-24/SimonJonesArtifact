import os

import roslaunch
import rospy

px4_launch = None
gazebo_launch = None
clover_services_launch = None
clover_model_launch = None

# no se
px4_launch_pid = 0
gazebo_launch_pid = 0
clover_services_launch_pid = 0
clover_model_launch_pid = 0

def launch_px4():
    """Launch px4 node."""
    pid = os.fork()
    # exit if parent process
    if pid > 0:
        px4_launch_pid = pid
        return
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    px4_launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [os.path.join("launch", "px4.launch")]
    )
    px4_launch.start()
    rospy.loginfo("started px4")

def launch_gazebo():
    """Launch gazebo node."""
    pid = os.fork()
    # exit if parent process
    if pid > 0:
        gazebo_launch_pid = pid
        return
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    gazebo_launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [os.path.join("launch", "gazebo.launch")]
    )
    gazebo_launch.start()
    rospy.loginfo("started gazebo")

def launch_clover_services():
    """Launch clover_services node."""
    pid = os.fork()
    # exit if parent process
    if pid > 0:
        clover_services_launch_pid = pid
        return
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    clover_services_launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [os.path.join("launch", "clover_services.launch")]
    )
    clover_services_launch.start()
    rospy.loginfo("started clover_services")

def launch_clover_model():
    """Launch clover_model node."""
    pid = os.fork()
    # exit if parent process
    if pid > 0:
        clover_model_launch_pid = pid
        return
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    clover_model_launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [os.path.join("launch", "clover_model.launch")]
    )
    clover_model_launch.start()
    rospy.loginfo("started clover_model")

def shutdown_px4():
    """Shut down px4 node."""
    if (px4_launch_pid is None):
        rospy.logerr("unable to shutdown node instance that has not been started")
    else:
        os.kill(px4_launch_pid)
        rospy.loginfo("shutdown px4")

def shutdown_gazebo():
    """Shut down gazebo node."""
    if (gazebo_launch_pid is None):
        rospy.logerr("unable to shutdown node instance that has not been started")
    else:
        os.kill(gazebo_launch_pid)
        rospy.loginfo("shutdown gazebo")

def shutdown_clover_services():
    """Shut down services node."""
    if (clover_services_launch_pid is None):
        rospy.logerr("unable to shutdown node instance that has not been started")
    else:
        os.kill(clover_services_launch_pid)
        rospy.loginfo("started clover_services")

def shutdown_clover_model():
    """Shut down model node."""
    if (clover_model_launch_pid is None):
        rospy.logerr("unable to shutdown node instance that has not been started")
    else:
        os.kill(clover_model_launch_pid)
        rospy.loginfo("started clover_model")
