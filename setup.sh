#! /usr/bin/env sh

# taken from https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html
. /opt/ros/rolling/setup.sh

# choose arbitrary domain ID
# https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Domain-ID.html
export ROS_DOMAIN_ID=21

# setup colcon, a tool for building and managing ROS2 workspaces and packages
# https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
. /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/rolling/
. /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
