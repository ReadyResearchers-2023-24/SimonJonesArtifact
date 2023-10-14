import rospy
from clover import srv
from std_srvs.srv import Trigger
from std_srvs.srv import Empty
import threading

# initialize rospy flight node
rospy.init_node('flight')

# initialize clover-related services
rospy.wait_for_service('/gazebo/reset_world')
reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# create shared mutex for reading console input
key_mutex = threading.Lock()
keypress_queue = []
run = True

class DroneMovements:
    def up():
        """Move up one meter."""
        navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)

    def down():
        """Move down one meter."""
        navigate(x=0, y=0, z=-1, frame_id='body', auto_arm=True)

    def left():
        """Move left one meter."""
        navigate(x=-1, y=0, z=0, frame_id='body', auto_arm=True)

    def right():
        """Move right one meter."""
        navigate(x=1, y=0, z=0, frame_id='body', auto_arm=True)

    def forward():
        """Move forward one meter."""
        navigate(x=0, y=1, z=0, frame_id='body', auto_arm=True)

    def backward():
        """Move backward one meter."""
        navigate(x=0, y=-1, z=1, frame_id='body', auto_arm=True)

    def reset():
        """Reset world."""
        reset_world()

    def land():
        """Land the drone."""
        land()


def ros_runner(lock):
    """Function for sending ros messages."""
    global run
    while True:
        # check if quit has been triggered
        lock.acquire()
        if not run:
            lock.release()
            break
        if (len(keypress_queue) > 0):
            # obtain next keypress from the queue
            next_keypress = keypress_queue.pop()
            # release the lock and let keyboard events be processed
            lock.release()
            # process commands
            if (next_keypress.lower() == "q"):
                lock.acquire()
                run = False
                lock.release()
            elif (next_keypress.lower() == " "):
                DroneMovements.up()
            elif (next_keypress.lower() == "x"):
                DroneMovements.down()
            elif (next_keypress.lower() == "a"):
                DroneMovements.left()
            elif (next_keypress.lower() == "d"):
                DroneMovements.right()
            elif (next_keypress.lower() == "w"):
                DroneMovements.forward()
            elif (next_keypress.lower() == "s"):
                DroneMovements.backward()
            elif (next_keypress.lower() == "r"):
                DroneMovements.reset()
            elif (next_keypress.lower() == "l"):
                DroneMovements.land()
        else:
            lock.release()


def keypress_runner(lock):
    """Function for handling keypresses."""
    global run
    # print documentation
    print("""\nPress one of the following keys to perform a command, followed by [ENTER].

        [SPACE] .. up
        [X] ...... down
        [A] ...... left
        [D] ...... right
        [W] ...... forward
        [S] ...... backward

        [Q] ...... quit
        [R] ...... reset world
        [L] ...... land

    """)
    while True:
        # check if quit has been triggered
        lock.acquire()
        if not run:
            lock.release()
            break
        lock.release()
        # read input
        c = input("> ")
        if (len(c) == 1):
            # save input to queue
            lock.acquire()
            keypress_queue.append(c)
            lock.release()


ros_thread = threading.Thread(target=ros_runner, args=(key_mutex,))
keypress_thread = threading.Thread(target=keypress_runner, args=(key_mutex,))

ros_thread.start()
keypress_thread.start()
