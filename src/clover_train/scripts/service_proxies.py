#!/usr/bin/env python3

import rospy

from clover import srv
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Empty
from std_srvs.srv import Trigger

arming = None
get_telemetry = None
land = None
navigate = None
navigate_global = None
reset_world = None
set_attitude = None
set_position = None
set_rates = None
set_velocity = None

def init():
    """Create service proxies"""
    # wait for /gazebo/reset_world to become available
    rospy.wait_for_service('/gazebo/reset_world')
    globals()["arming"] = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    globals()["reset_world"] = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    # https://github.com/CopterExpress/clover/blob/master/clover/srv/GetTelemetry.srv
    globals()["get_telemetry"] = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    globals()["navigate"] = rospy.ServiceProxy('navigate', srv.Navigate)
    globals()["navigate_global"] = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
    globals()["set_position"] = rospy.ServiceProxy('set_position', srv.SetPosition)
    globals()["set_velocity"] = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
    globals()["set_attitude"] = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
    globals()["set_rates"] = rospy.ServiceProxy('set_rates', srv.SetRates)
    globals()["land"] = rospy.ServiceProxy('land', Trigger)
