#!/usr/bin/env python3

import os

import roslaunch
import rosnode
import rospkg
import rospy
import time
import subprocess

from geometry_msgs.msg import Pose


NODE_KILL_TIMEOUT = 30  # seconds

clover_simulation_launch = None


def launch_clover_simulation(gazebo_world_filepath: str = None, gui: bool = True, clover_pose: Pose = None) -> None:
    """Launch all nodes related to clover_simulation."""
    global clover_simulation_launch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # find clover_simulation launch file
    rospack = rospkg.RosPack()
    clover_simulation_path = rospack.get_path("clover_simulation")
    clover_simulation_launch_path = os.path.join(
        clover_simulation_path, "launch", "simulator.launch"
    )
    # create list with cli arguments
    cli_args = [clover_simulation_launch_path]
    # enable/disable GUI
    cli_args.append(f"gui:={str(gui).lower()}")
    cli_args.append("use_master_node:=true")
    # if supplied, add filepath for gazebo .world file
    if gazebo_world_filepath is not None:
        cli_args.append(f"gazebo_world_filepath:={gazebo_world_filepath}")
    # if supplied, forward custom positions to spawn_drone launch file via
    # simulator launch file
    if clover_pose is not None:
        cli_args.append("use_custom_clover_position:=true")
        cli_args.append(f"clover_x_position:={clover_pose.position.x}")
        cli_args.append(f"clover_y_position:={clover_pose.position.y}")
        cli_args.append(f"clover_z_position:={clover_pose.position.z}")
    # construct launch parent object for starting launch file
    clover_simulation_launch = roslaunch.parent.ROSLaunchParent(uuid, [(cli_args[0], cli_args[1:])])
    clover_simulation_launch.start()


def kill_clover_simulation() -> None:
    """Kill all nodes related to clover_simulation."""
    if clover_simulation_launch is not None:
        clover_simulation_launch.shutdown()
    subprocess.run(["killall", "gzserver"])
    subprocess.run(["killall", "gzclient"])
    # do not kill roscore processes
    nodes_to_not_kill = {"/rosout", "/clover_train"}
    # take set difference between existing nodes and nodes to not kill
    nodes_to_kill = list(set(rosnode.get_node_names()) - nodes_to_not_kill)
    # kill nodes
    rosnode.kill_nodes(nodes_to_kill)
    # wait until all nodes are killed
    t0 = time.time()
    rospy.loginfo("[kill_clover_simulation] waiting until all nodes are killed.")
    while len(set(rosnode.get_node_names()).intersection(set(nodes_to_kill))) > 0:
        # if timeout is passed, we will assume we killed the nodes properly
        # see https://github.com/ros-simulation/gazebo_ros_pkgs/issues/751#issuecomment-635720144
        if time.time() - t0 > NODE_KILL_TIMEOUT:
            break
        # kill nodes
        rosnode.kill_nodes(nodes_to_kill)
    rospy.loginfo(
        f"[kill_clover_simulation] successfully killed nodes {nodes_to_kill}."
    )
