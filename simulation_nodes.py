import rospy
import roslaunch

px4_launch = None
gazebo_launch = None
clover_services_launch = None
clover_model_launch = None

def launch_px4():
    """Launch px4 node."""
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    px4_launch = roslaunch.parent.ROSLaunchParent(uuid, ["./px4.launch"])
    px4_launch.start()
    rospy.loginfo("started px4")

def launch_gazebo():
    """Launch gazebo node."""
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    gazebo_launch = roslaunch.parent.ROSLaunchParent(uuid, ["./gazebo.launch"])
    gazebo_launch.start()
    rospy.loginfo("started gazebo")

def launch_clover_services():
    """Launch clover_services node."""
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    clover_services_launch = roslaunch.parent.ROSLaunchParent(uuid, ["./clover_services.launch"])
    clover_services_launch.start()
    rospy.loginfo("started clover_services")

def launch_clover_model():
    """Launch clover_model node."""
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    clover_model_launch = roslaunch.parent.ROSLaunchParent(uuid, ["./clover_model.launch"])
    clover_model_launch.start()
    rospy.loginfo("started clover_model")

def shutdown_px4():
    """Shut down px4 node."""
    if (px4_launch is None):
        rospy.logerr("unable to shutdown node instance that has not been started")
    else:
        px4_launch.shutdown()
        rospy.loginfo("shutdown px4")

def shutdown_gazebo():
    """Shut down gazebo node."""
    if (gazebo_launch is None):
        rospy.logerr("unable to shutdown node instance that has not been started")
    else:
        gazebo_launch.shutdown()
        rospy.loginfo("shutdown gazebo")

def shutdown_clover_services():
    """Shut down services node."""
    if (clover_services_launch is None):
        rospy.logerr("unable to shutdown node instance that has not been started")
    else:
        clover_services_launch.shutdown()
        rospy.loginfo("started clover_services")

def shutdown_clover_model():
    """Shut down model node."""
    if (clover_model_launch is None):
        rospy.logerr("unable to shutdown node instance that has not been started")
    else:
        clover_model_launch.shutdown()
        rospy.loginfo("started clover_model")

def set_up_service_proxies():
    rospy.wait_for_service('/gazebo/reset_world')
    arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
    set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
    set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
    set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
    set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
    land = rospy.ServiceProxy('land', Trigger)

# FIXME: create static class for this
