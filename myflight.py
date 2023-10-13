import rospy
import numpy as np
import tensorflow as tf

from clover import srv
from std_srvs.srv import Trigger
from std_srvs.srv import Empty
from tensorflow import keras
from tensorflow.keras import layers

gamma = 0.99 # discount factor for past rewards
epsilon = np.finfo(np.float32).eps.item() # smallest number such that 1.0 + eps != 1.0

num_inputs = 4
num_actions = 2
num_hidden = 128

inputs = layers.Input(shape=(num_inputs,))
common = layers.Dense(num_hidden, activation="relu")(inputs)
action = layers.Dense(num_actions, activation="softmax")(common)
critic = layers.Dense(1)(common)

model = keras.Model(inputs=inputs, outputs=[action, critic])

rospy.init_node('flight')

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

while True:
    reset_world()

    navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
    # Wait for 5 seconds
    rospy.sleep(5)

    print(get_telemetry())
    set_attitude(roll=1.0, pitch=1.0, yaw=1.0, thrust=1.0)
    rospy.sleep(1)

    print('Fly forward 1 m')
    navigate(x=-3, y=0, z=0, frame_id='body')

    # Wait for 5 seconds
    rospy.sleep(5)

    print('Perform landing')
    land()
