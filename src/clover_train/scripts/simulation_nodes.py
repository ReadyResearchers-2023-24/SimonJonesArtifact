#!/usr/bin/env python3

import os

import roslaunch
import rospy
import rospkg
import rosnode


def launch_clover_simulation(gazebo_world_filepath: str = None) -> None:
    """Launch all nodes related to clover_simulation."""
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
    # if supplied, add filepath for gazebo .world file
    if gazebo_world_filepath is not None:
        cli_args.append(f"gazebo_world_filepath:={gazebo_world_filepath}")
    # construct launch parent object for starting launch file
    this_launch = roslaunch.parent.ROSLaunchParent(uuid, cli_args)
    this_launch.start()


def kill_clover_simulation() -> None:
    """Kill all nodes related to clover_simulation."""
    # do not kill roscore processes
    nodes_to_not_kill = {"/rosout", "/clover_train"}
    # take set difference between existing nodes and nodes to not kill
    nodes_to_kill = list(set(rosnode.get_node_names()) - nodes_to_not_kill)
    # kill nodes
    rosnode.kill_nodes(nodes_to_kill)
    # wait until all nodes are killed
    r = rospy.Rate(1)
    rospy.loginfo("[kill_clover_simulation] waiting until all nodes are killed.")
    while len(set(rosnode.get_node_names()).intersection(set(nodes_to_kill))) > 0:
        r.sleep()
    rospy.loginfo(
        f"[kill_clover_simulation] successfully killed nodes {nodes_to_kill}."
    )
