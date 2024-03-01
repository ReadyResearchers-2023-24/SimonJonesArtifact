#!/usr/bin/env python3

from typing import List

import pcg
import pcg_gazebo
import rospy
import std_msgs
import os


def get_pcg_map_metadata(req: pcg.srv.GetPcgMapMetadataRequest) -> pcg.srv.GetPcgMapMetadataResponse:
    """Get the metadata for a procedurally-generated map given a filepath."""
    free_positions: List[std_msgs.msg.Point] = []
    # FIXME: return free positions
    if os.path.exists(req.world_sdf_filepath):
        world = pcg_gazebo.generators.WorldGenerator()
        world.init_from_sdf(req.world_sdf_filepath)
        return pcg.srv.GetPcgMapMetadataResponse(free_positions)
    else:
        return None

if __name__=="__main__":
    rospy.init_node("broadcast")
    s = rospy.Service("get_pcg_map_metadata", pcg.srv.GetPcgMapMetadata, get_pcg_map_metadata)
    rospy.spin()
