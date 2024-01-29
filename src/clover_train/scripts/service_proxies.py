#!/usr/bin/env python3

import rospy

from clover import srv
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Empty
from std_srvs.srv import Trigger
from gazebo_msgs.srv import GetModelState, DeleteModel, SpawnModel

arming = None
delete_model = None
get_model_state = None
get_telemetry = None
land = None
navigate = None
navigate_global = None
pause_physics = None
reset_world = None
set_attitude = None
set_position = None
set_rates = None
set_velocity = None
spawn_urdf_model = None
unpause_physics = None

def init():
    """Create service proxies"""
    # wait for /gazebo/reset_world to become available
    rospy.wait_for_service('/gazebo/reset_world')
    # https://github.com/CopterExpress/clover/blob/master/clover/srv/GetTelemetry.srv
    globals()["arming"] = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    globals()["delete_model"] = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    globals()["get_model_state"] = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    globals()["get_telemetry"] = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    globals()["land"] = rospy.ServiceProxy('land', Trigger)
    globals()["navigate"] = rospy.ServiceProxy('navigate', srv.Navigate)
    globals()["navigate_global"] = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
    globals()["pause_physics"] = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
    globals()["reset_world"] = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    globals()["set_attitude"] = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
    globals()["set_position"] = rospy.ServiceProxy('set_position', srv.SetPosition)
    globals()["set_rates"] = rospy.ServiceProxy('set_rates', srv.SetRates)
    globals()["set_velocity"] = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
    globals()["spawn_urdf_model"] = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    globals()["unpause_physics"] = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
