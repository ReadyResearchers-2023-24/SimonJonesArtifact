#!/usr/bin/env python3

import rospy

from clover import srv as clover_srv
from mavros_msgs.srv import CommandLong
from std_srvs.srv import Empty
from std_srvs.srv import Trigger

get_telemetry = None
land = None
mavros_command = None
navigate = None
navigate_global = None
pause_physics = None
reset_world = None
set_attitude = None
set_position = None
set_rates = None
set_velocity = None
simulation_killswitch = None
unpause_physics = None


def init():
    """Create service proxies"""
    # wait for /gazebo/reset_world to become available
    rospy.wait_for_service("/gazebo/reset_world")
    # https://github.com/CopterExpress/clover/blob/master/clover/srv/GetTelemetry.srv
    globals()["get_telemetry"] = rospy.ServiceProxy(
        "get_telemetry", clover_srv.GetTelemetry
    )
    globals()["land"] = rospy.ServiceProxy("land", Trigger)
    globals()["mavros_command"] = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
    globals()["navigate"] = rospy.ServiceProxy("navigate", clover_srv.Navigate)
    globals()["navigate_global"] = rospy.ServiceProxy(
        "navigate_global", clover_srv.NavigateGlobal
    )
    globals()["pause_physics"] = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
    globals()["reset_world"] = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    globals()["set_attitude"] = rospy.ServiceProxy(
        "set_attitude", clover_srv.SetAttitude
    )
    globals()["set_position"] = rospy.ServiceProxy(
        "set_position", clover_srv.SetPosition
    )
    globals()["set_rates"] = rospy.ServiceProxy("set_rates", clover_srv.SetRates)
    globals()["set_velocity"] = rospy.ServiceProxy(
        "set_velocity", clover_srv.SetVelocity
    )
    globals()["simulation_killswitch"] = rospy.ServiceProxy(
        "simulation_killswitch", Empty
    )
    globals()["unpause_physics"] = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
