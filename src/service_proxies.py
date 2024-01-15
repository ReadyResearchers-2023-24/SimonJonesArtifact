import rospy

from clover import srv
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Empty
from std_srvs.srv import Trigger

arming = None
reset_world = None
get_telemetry = None
navigate = None
navigate_global = None
set_position = None
set_velocity = None
set_attitude = None
set_rates = None
land = None

def init(self):
    """Create service proxies"""
    # wait for /gazebo/reset_world to become available
    rospy.wait_for_service('/gazebo/reset_world')
    arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    # https://github.com/CopterExpress/clover/blob/master/clover/srv/GetTelemetry.srv
    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
    set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
    set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
    set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
    set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
    land = rospy.ServiceProxy('land', Trigger)
