#!/usr/bin/env python3

import copy
import math
import numpy as np
import rospy
import service_proxies
import simulation_nodes
import tensorflow as tf

from geometry_msgs.msg import PoseStamped, TwistStamped
from tensorflow.keras import layers
from threading import Lock, Thread
from typing import List, Any
from dataclasses import dataclass, fields, asdict

rospy.init_node("clover_train")


def count_dataclass_fields(the_dataclass) -> int:
    """Count the number of fields in a dataclass."""
    dummy_instance = the_dataclass()
    return len([field.name for field in fields(dummy_instance)])


def dataclass_to_list(the_dataclass_instance) -> List[Any]:
    return list(asdict(the_dataclass).values())


# define state space
@dataclass
class State:
    px: float = 0.0
    py: float = 0.0
    pz: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0


num_states = count_dataclass_fields(State)

# initialize global state variable
state = State()


# define action space
@dataclass
class Action:
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0


num_actions = count_dataclass_fields(Action)

rangefinder_range: float = 6.0  # meter
action_min: float = -rangefinder_range  # meter
action_max: float = rangefinder_range  # meter

time_step_ms = 100

# note: currently, rospy will only allow one callback instance at a time to be run
# this omits any worries about multiple callbacks loading up to get access to the mutex

# FIXME: create wrapper for subscriber/mutex combo

state_mutex = Lock()

max_mutex_misses_before_warning = 10


def warn_if_too_many_mutex_misses(mutex_misses):
    if mutex_misses > max_mutex_misses_before_warning:
        rospy.logerr(
            f"callback has failed to acquire mutex more than {max_mutex_misses_before_warning} times"
        )


# times the callback failed to acquire the mutex
velocity_callback_mutex_misses = 0


def velocity_callback(velocity):
    global velocity_callback_mutex_misses
    mutex_acquired = state_mutex.acquire(blocking=False)
    # Quickly terminate if mutex was not acquired. This is banking on the fact
    # that ros does not allow multiple instances of the same callback function.
    # Otherwise, we would have to use atomic types.
    if not mutex_acquired:
        velocity_callback_mutex_misses += 1
        warn_if_too_many_mutex_misses(velocity_callback_mutex_misses)
        # take mod of misses; warn every time it maxes out
        velocity_callback_mutex_misses %= max_mutex_misses_before_warning
        return
    state.vx = velocity.twist.linear.x
    state.vy = velocity.twist.linear.y
    state.vz = velocity.twist.linear.z
    state.wx = velocity.twist.angular.x
    state.wy = velocity.twist.angular.y
    state.wz = velocity.twist.angular.z
    state_mutex.release()


def velocity_listener():
    # subscribe to mavros's pose topic
    rospy.Subscriber(
        "/mavros/local_position/velocity_local", TwistStamped, velocity_callback
    )
    rospy.spin()


velocity_thread = Thread(target=velocity_listener, args=())
velocity_thread.start()

# times the callback failed to acquire the mutex
local_position_callback_mutex_misses = 0


def local_position_callback(local_position):
    global local_position_callback_mutex_misses
    mutex_acquired = state_mutex.acquire(blocking=False)
    # Quickly terminate if mutex was not acquired. This is banking on the fact
    # that ros does not allow multiple instances of the same callback function.
    # Otherwise, we would have to use atomic types.
    if not mutex_acquired:
        local_position_callback_mutex_misses += 1
        warn_if_too_many_mutex_misses(local_position_callback_mutex_misses)
        # take mod of misses; warn every time it maxes out
        local_position_callback_mutex_misses %= max_mutex_misses_before_warning
        return
    # position
    state.px = local_position.pose.position.x
    state.py = local_position.pose.position.y
    state.pz = local_position.pose.position.z
    # quaternion
    state.qx = local_position.pose.orientation.x
    state.qy = local_position.pose.orientation.y
    state.qz = local_position.pose.orientation.z
    state.qw = local_position.pose.orientation.w
    state_mutex.release()


def local_position_listener():
    # subscribe to mavros's pose topic
    rospy.Subscriber(
        "/mavros/local_position/pose", PoseStamped, local_position_callback
    )
    rospy.spin()


local_position_thread = Thread(target=local_position_listener, args=())
local_position_thread.start()


class OUActionNoise:
    def __init__(self, mean, std_deviation, theta=0.15, dt=1e-2, x_initial=None):
        self.theta = theta
        self.mean = mean
        self.std_dev = std_deviation
        self.dt = dt
        self.x_initial = x_initial
        self.reset()

    def __call__(self):
        # Formula taken from https://www.wikipedia.org/wiki/Ornstein-Uhlenbeck_process.
        x = (
            self.x_prev
            + self.theta * (self.mean - self.x_prev) * self.dt
            + self.std_dev * np.sqrt(self.dt) * np.random.normal(size=self.mean.shape)
        )
        # Store x into x_prev
        # Makes next noise dependent on current one
        self.x_prev = x
        return x

    def reset(self):
        if self.x_initial is not None:
            self.x_prev = self.x_initial
        else:
            self.x_prev = np.zeros_like(self.mean)


class Buffer:
    def __init__(self, buffer_capacity=100000, batch_size=64):
        # Number of "experiences" to store at max
        self.buffer_capacity = buffer_capacity
        # Num of tuples to train on.
        self.batch_size = batch_size

        # Its tells us num of times record() was called.
        self.buffer_counter = 0

        # Instead of list of tuples as the exp.replay concept go
        # We use different np.arrays for each tuple element
        self.state_buffer = np.zeros((self.buffer_capacity, num_states))
        self.action_buffer = np.zeros((self.buffer_capacity, num_actions))
        self.reward_buffer = np.zeros((self.buffer_capacity, 1))
        self.next_state_buffer = np.zeros((self.buffer_capacity, num_states))

    # Takes (s,a,r,s') obervation tuple as input
    def record(self, obs_tuple: Tuple[State, Action, float, State]):
        # Set index to zero if buffer_capacity is exceeded,
        # replacing old records
        index = self.buffer_counter % self.buffer_capacity

        self.state_buffer[index] = dataclass_to_list(obs_tuple[0])
        self.action_buffer[index] = dataclass_to_list(obs_tuple[1])
        self.reward_buffer[index] = obs_tuple[2]
        self.next_state_buffer[index] = dataclass_to_list(obs_tuple[3])

        self.buffer_counter += 1

    # Eager execution is turned on by default in TensorFlow 2. Decorating with tf.function allows
    # TensorFlow to build a static graph out of the logic and computations in our function.
    # This provides a large speed up for blocks of code that contain many small TensorFlow operations such as this one.
    @tf.function
    def update(
        self,
        state_batch,
        action_batch,
        reward_batch,
        next_state_batch,
    ):
        # Training and updating Actor & Critic networks.
        # See Pseudo Code.
        with tf.GradientTape() as tape:
            target_actions = target_actor(next_state_batch, training=True)
            y = reward_batch + gamma * target_critic(
                [next_state_batch, target_actions], training=True
            )
            critic_value = critic_model([state_batch, action_batch], training=True)
            critic_loss = tf.math.reduce_mean(tf.math.square(y - critic_value))

        critic_grad = tape.gradient(critic_loss, critic_model.trainable_variables)
        critic_optimizer.apply_gradients(
            zip(critic_grad, critic_model.trainable_variables)
        )

        with tf.GradientTape() as tape:
            actions = actor_model(state_batch, training=True)
            critic_value = critic_model([state_batch, actions], training=True)
            # Used `-value` as we want to maximize the value given
            # by the critic for our actions
            actor_loss = -tf.math.reduce_mean(critic_value)

        actor_grad = tape.gradient(actor_loss, actor_model.trainable_variables)
        actor_optimizer.apply_gradients(
            zip(actor_grad, actor_model.trainable_variables)
        )

    # We compute the loss and update parameters
    def learn(self):
        # Get sampling range
        record_range = min(self.buffer_counter, self.buffer_capacity)
        # Randomly sample indices
        batch_indices = np.random.choice(record_range, self.batch_size)

        # Convert to tensors
        state_batch = tf.convert_to_tensor(self.state_buffer[batch_indices])
        action_batch = tf.convert_to_tensor(self.action_buffer[batch_indices])
        reward_batch = tf.convert_to_tensor(self.reward_buffer[batch_indices])
        reward_batch = tf.cast(reward_batch, dtype=tf.float32)
        next_state_batch = tf.convert_to_tensor(self.next_state_buffer[batch_indices])

        self.update(state_batch, action_batch, reward_batch, next_state_batch)


# This update target parameters slowly
# Based on rate `tau`, which is much less than one.
@tf.function
def update_target(target_weights, weights, tau):
    for a, b in zip(target_weights, weights):
        a.assign(b * tau + a * (1 - tau))


def get_actor():
    # Initialize weights
    last_init_linear = tf.random_uniform_initializer(
        minval=-rangefinder_range, maxval=rangefinder_range
    )

    inputs = layers.Input(shape=(num_states,))
    out = layers.Dense(256, activation="relu")(inputs)
    out = layers.Dense(256, activation="relu")(out)
    # initialize linear outputs for actions with range [+x, -x]
    # put in list (to make extensible in case of adding more actions)
    outputs_list = [
        layers.Dense(3, activation="linear", kernel_initializer=last_init_linear)(out),
    ]
    outputs = layers.concatenate(outputs_list)

    # scale the outputs, given that they're coming from
    # the basis of an activation function
    output_scale = tf.convert_to_tensor([action_max, action_max, action_max])
    outputs = tf.math.multiply(outputs, output_scale)
    model = tf.keras.Model(inputs, outputs)
    return model


def get_critic():
    # State as input
    state_input = layers.Input(shape=(num_states))
    state_out = layers.Dense(16, activation="relu")(state_input)
    state_out = layers.Dense(32, activation="relu")(state_out)

    # Action as input
    action_input = layers.Input(shape=(num_actions))
    action_out = layers.Dense(32, activation="relu")(action_input)

    # Both are passed through seperate layer before concatenating
    concat = layers.Concatenate()([state_out, action_out])

    out = layers.Dense(256, activation="relu")(concat)
    out = layers.Dense(256, activation="relu")(out)
    outputs = layers.Dense(num_actions)(out)

    # Outputs single value for give state-action
    model = tf.keras.Model([state_input, action_input], outputs)

    return model


def policy(state: State, noise_object: OUActionNoise):
    sampled_actions = tf.squeeze(actor_model(dataclass_to_list(state)))
    print("state: ", state)

    # create tensor with noise
    noise_array = noise_object()

    # Adding noise to action
    sampled_actions = sampled_actions.numpy() + noise_array
    lower_bounds = [action_min, action_min, action_min]
    upper_bounds = [action_max, action_max, action_max]

    # We make sure action is within bounds
    legal_action = np.clip(sampled_actions, lower_bounds, upper_bounds)

    return np.squeeze(legal_action)


def episode_calculate_reward_metric(local_state: State) -> float:
    # NOTE: temporary: for now, try to hover at (x,y,z) = (0,1,0)
    desired_pos = {"x": 0, "y": 0, "z": 1}
    distance_from_desired_pos = (
        (local_state.px - desired_pos["x"]) ** 2
        + (local_state.py - desired_pos["y"]) ** 2
        + (local_state.pz - desired_pos["z"]) ** 2
    ) ** (1 / 2)
    reward = -distance_from_desired_pos + 100 * (
        math.e ** (-((local_state.pz - 1) ** 2)) - 1
    )
    return reward


def quaternion_to_euler_angles(
    qx: float, qy: float, qz: float, qw: float
) -> Tuple[float, float, float]:
    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = math.sqrt(1 + 2 * (qw * qy - qx * qz))
    cosp = math.sqrt(1 - 2 * (qw * qy - qx * qz))
    pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


def episode_calculate_if_done(local_state: State):
    # NOTE: temporary for now, done if 10m away from goal
    desired_pos = {"x": 0.0, "y": 1.0, "z": 0.0}
    print("[episode_calculate_if_done] trying to acquire mutex...")
    print("[episode_calculate_if_done] mutex acquired!")
    distance_from_desired_pos = (
        (local_state.px - desired_pos["x"]) ** 2
        + (local_state.py - desired_pos["y"]) ** 2
        + (local_state.pz - desired_pos["z"]) ** 2
    ) ** (1 / 2)
    print("distance from desired position: ", distance_from_desired_pos)
    # get current angular orientation
    (roll, pitch, yaw) = quaternion_to_euler_angles(
        local_state.qx, local_state.qy, local_state.qz, local_state.qw
    )
    print(f"[episode_calculate_if_done] roll: {roll} pitch: {pitch} yaw: {yaw}")
    done = (
        False
        or distance_from_desired_pos > 10.0
        or abs(roll) > math.pi
        or abs(pitch) > math.pi
    )
    return done


def episode_reset_and_grab_state():
    global state, state_mutex
    # set quadcopter attitude to initial state
    # https://docs.ros.org/en/melodic/api/clover/html/srv/SetAttitude.html
    service_proxies.set_attitude(pitch=0, roll=0, yaw=0, thrust=0, auto_arm=True)
    # FIXME: wrap this if it works
    # force disarm the quadcopter
    service_proxies.mavros_command(
        broadcast=False,
        command=400,
        confirmation=0,
        param2=21196,
    )
    service_proxies.reset_world()
    telemetry_class = service_proxies.get_telemetry()
    # pull out parts of the state
    # that we want to know from telemetry
    state_mutex.acquire()
    local_state = copy.deepcopy(state)
    state_mutex.release()
    return local_state


def episode_take_action(action: Action) -> Tuple[State, float, bool]:
    global state
    # coordinate systems in clover:
    # https://clover.coex.tech/en/frames.html#coordinate-systems-frames

    # FIXME: lag in clover ROS causes this to not work

    print("[episode_take_action]", action)
    # navigate a distance relative to the quadcopter's frame
    # The 'body' frame remains upright regardless of
    # the quadcopter's orientation.
    service_proxies.navigate(
        x=action.dx,
        y=action.dy,
        z=action.dz,
        speed=1,
        frame_id="body",
        auto_arm=True,
    )
    # wait until target is reached
    tolerance = 0.2  # meters
    # source: https://clover.coex.tech/en/snippets.html#navigate_wait
    while not rospy.is_shutdown():
        # get current position relative to desired position
        telem = get_telemetry(frame_id="navigate_target")
        if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
            break
        rospy.sleep(0.2)
    state_mutex.acquire()
    local_state = copy.deepcopy(state)
    state_mutex.release()
    reward = episode_calculate_reward_metric(local_state)
    done = episode_calculate_if_done(local_state)
    return (local_state, reward, done)


std_dev = 0.2
ou_noise = OUActionNoise(
    mean=np.zeros(num_actions), std_deviation=float(std_dev) * np.ones(num_actions)
)

actor_model = get_actor()
critic_model = get_critic()

target_actor = get_actor()
target_critic = get_critic()

# Making the weights equal initially
target_actor.set_weights(actor_model.get_weights())
target_critic.set_weights(critic_model.get_weights())

# Learning rate for actor-critic models
critic_lr = 0.002
actor_lr = 0.001

critic_optimizer = tf.keras.optimizers.Adam(critic_lr)
actor_optimizer = tf.keras.optimizers.Adam(actor_lr)

total_episodes = 200

# Discount factor for future rewards
gamma = 0.99
# Used to update target networks
tau = 0.005

buffer = Buffer(50000, 64)

# To store reward history of each episode
ep_reward_list = []
# To store average reward history of last few episodes
avg_reward_list = []

# NOTE: set to true to launch nodes from this process
launch_simulation_nodes_manually = False
if launch_simulation_nodes_manually:
    simulation_nodes.launch_px4()
    simulation_nodes.launch_gazebo()
    simulation_nodes.launch_clover_services()
    simulation_nodes.launch_clover_model()

# set up ROS related handles
service_proxies.init()

for ep in range(total_episodes):
    prev_state = episode_reset_and_grab_state()
    episodic_reward = 0

    # set loop rate of 100Hz
    # this is handled by ROS to ensure consistent steps
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        print("state0:", prev_state)
        tf_prev_state = tf.expand_dims(tf.convert_to_tensor(prev_state), 0)

        action = policy(tf_prev_state, ou_noise)
        print("[TRACE] policy calculated")
        # Recieve state and reward from environment.
        local_state, reward, done = episode_take_action(action)
        print("[TRACE] action taken")

        buffer.record((prev_state, action, reward, local_state))
        episodic_reward += reward
        print("[TRACE] recorded")

        buffer.learn()
        update_target(target_actor.variables, actor_model.variables, tau)
        update_target(target_critic.variables, critic_model.variables, tau)
        print("[TRACE] learned and updated")

        # End this episode when `done` is True
        if done:
            break

        prev_state = local_state
        # sleep governed by rospy.Rate to ensure consistent loop intervals
        r.sleep()
        print("[TRACE] slept")

    ep_reward_list.append(episodic_reward)

    # Mean of last 40 episodes
    avg_reward = np.mean(ep_reward_list[-40:])
    print(f"Episode * {ep} * Avg Reward is ==> {avg_reward}")
    avg_reward_list.append(avg_reward)

# Save the weights
actor_model.save_weights("ddpg_actor.h5")
critic_model.save_weights("ddpg_critic.h5")

target_actor.save_weights("ddpg_target_actor.h5")
target_critic.save_weights("ddpg_target_critic.h5")

rospy.signal_shutdown(f"Succesfully {total_episodes}. Signaling shut down.")

local_position_thread.join()
