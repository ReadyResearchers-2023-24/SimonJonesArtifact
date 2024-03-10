#!/usr/bin/env python3

import copy
import datetime
import math
import numpy as np
import os
import rospkg
import rospy
import service_proxies
import simulation_nodes
import tensorflow as tf
import util

from geometry_msgs.msg import PoseStamped, TwistStamped
from tensorflow.keras import layers
from threading import Lock, Thread
from typing import Tuple
from dataclasses import dataclass
from pymavlink import mavutil
from mavros_msgs import msg as mavros_msg
from sensor_msgs import msg as sensor_msg

rospy.init_node("clover_train")


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
    range_0: float = 0.0
    range_1: float = 0.0
    range_2: float = 0.0
    range_3: float = 0.0
    range_4: float = 0.0
    range_5: float = 0.0
    range_6: float = 0.0
    range_7: float = 0.0
    range_8: float = 0.0
    range_9: float = 0.0


num_states = util.count_dataclass_fields(State)

# initialize global state variable
state = State()


# define action space as spherical coordinate system about body
@dataclass
class Action:
    r: float = 0.0  # distance measured from origin
    theta: float = 0.0  # angle measured from +z axis toward z=0 plane
    phi: float = 0.0  # angle measured from +x axis right handedly about z axis


num_actions = util.count_dataclass_fields(Action)

rangefinder_range: float = 6.0  # meter

# threshold for deciding when drone has reached desired position
action_movement_threshold: float = 0.1  # meter

# range of action space (to be used to scale action tensors)
# move at least twice the detection threshold for movement
action_r_min: float = action_movement_threshold * 2  # meter
action_theta_min: float = 0  # radian
action_phi_min: float = 0  # radian

action_r_max: float = rangefinder_range  # meter
action_theta_max: float = math.pi  # radian
action_phi_max: float = 2 * math.pi  # radian

# define list of worlds for each part of the cirriculum
# FIXME: grab from clover_broadcast package instead
rospack = rospkg.RosPack()
clover_simulation_path = rospack.get_path("pcg")
world_file_base_path = os.path.join(
    clover_simulation_path, "resources", "worlds"
)
cirriculum_worlds = [
    os.path.join(
        world_file_base_path, "2-rectangles-walls.world"
    ),
    os.path.join(
        world_file_base_path, "3-rectangles-walls.world"
    ),
    os.path.join(
        world_file_base_path, "4-rectangles-walls.world"
    ),
    os.path.join(
        world_file_base_path, "5-rectangles-walls.world"
    ),
    os.path.join(
        world_file_base_path, "6-rectangles-walls.world"
    ),
    os.path.join(
        world_file_base_path, "7-rectangles-walls.world"
    ),
    os.path.join(
        world_file_base_path, "8-rectangles-walls.world"
    ),
    os.path.join(
        world_file_base_path, "9-rectangles-walls.world"
    ),
    os.path.join(
        world_file_base_path, "10-rectangles-walls.world"
    ),
    os.path.join(
        world_file_base_path, "11-rectangles-walls.world"
    ),
]

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
    mutex_acquired = state_mutex.acquire(blocking=True)
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
    mutex_acquired = state_mutex.acquire(blocking=True)
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


# times the callback failed to acquire the mutex
rangefinder_callback_mutex_misses = 0


# FIXME: maybe use separate cpp module to transform range to be along the body frame?
# FIXME: maybe transform range measurements here?
def rangefinder_i_callback(i):
    def rangefinder_callback(range):
        global rangefinder_callback_mutex_misses
        mutex_acquired = state_mutex.acquire(blocking=True)
        # Quickly terminate if mutex was not acquired. This is banking on the fact
        # that ros does not allow multiple instances of the same callback function.
        # Otherwise, we would have to use atomic types.
        if not mutex_acquired:
            rangefinder_callback_mutex_misses += 1
            warn_if_too_many_mutex_misses(rangefinder_callback_mutex_misses)
            # take mod of misses; warn every time it maxes out
            rangefinder_callback_mutex_misses %= max_mutex_misses_before_warning
            return
        # set current range attribute on state
        setattr(state, f"range_{i}", range.range)
        state_mutex.release()

    return rangefinder_callback


def rangefinder_listener():
    # subscribe to mavros's pose topic
    for i in range(10):
        rospy.Subscriber(
            f"/rangefinder_{i}/range", sensor_msg.Range, rangefinder_i_callback(i)
        )
    rospy.spin()


rangefinder_thread = Thread(target=rangefinder_listener, args=())
rangefinder_thread.start()


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

        self.state_buffer[index] = util.dataclass_to_list(obs_tuple[0])
        self.action_buffer[index] = util.dataclass_to_list(obs_tuple[1])
        self.reward_buffer[index] = obs_tuple[2]
        self.next_state_buffer[index] = util.dataclass_to_list(obs_tuple[3])

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
    relu_initializer_uniform = tf.random_uniform_initializer(minval=0, maxval=1)
    relu_initializer_towards_zero = tf.random_uniform_initializer(minval=0, maxval=0.2)

    inputs = layers.Input(shape=(num_states,))
    out = layers.Dense(256, activation="relu")(inputs)
    out = layers.Dense(256, activation="relu")(out)
    # initialize relu outputs for actions with range [0, x]
    # put in list (to make extensible in case of adding more actions)
    outputs_list = [
        layers.Dense(1, activation="relu", kernel_initializer=relu_initializer_uniform)(
            out
        ),
        layers.Dense(
            1, activation="relu", kernel_initializer=relu_initializer_towards_zero
        )(out),
        layers.Dense(1, activation="relu", kernel_initializer=relu_initializer_uniform)(
            out
        ),
    ]
    outputs = layers.concatenate(outputs_list)

    # scale the outputs, given that they're coming from
    # the basis of an activation function
    output_scale = tf.convert_to_tensor(
        [action_r_max, action_theta_max, action_phi_max]
    )
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


def policy(state, noise_object: OUActionNoise) -> Action:
    sampled_actions = tf.squeeze(actor_model(state))
    print("state: ", state)

    # create tensor with noise
    noise_array = noise_object()

    # Adding noise to action
    sampled_actions = sampled_actions.numpy() + noise_array
    print(noise_array)
    lower_bounds = [action_r_min, action_theta_min, action_phi_min]
    upper_bounds = [action_r_max, action_theta_max, action_phi_max]

    # We make sure action is within bounds
    legal_action = np.clip(sampled_actions, lower_bounds, upper_bounds)

    return Action(*np.squeeze(legal_action))


def episode_calculate_reward_metric(local_state: State, timeout_passed: bool) -> float:
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
    if timeout_passed:
        reward -= 1000
    return reward


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
    (roll, pitch, yaw) = util.quaternion_to_euler_angles(
        local_state.qx, local_state.qy, local_state.qz, local_state.qw
    )
    print(f"[episode_calculate_if_done] roll: {roll} pitch: {pitch} yaw: {yaw}")
    done = (
        False
        or distance_from_desired_pos > 10.0
        or abs(roll) > math.pi / 2
        or abs(pitch) > math.pi / 2
    )
    return done


def is_armed() -> bool:
    mavros_state = rospy.wait_for_message("mavros/state", mavros_msg.State)
    return mavros_state.armed


def wait_until_disarmed() -> None:
    rospy.loginfo("[wait_until_disarmed] entered")
    while not rospy.is_shutdown():
        if not is_armed():
            rospy.loginfo("[wait_until_disarmed] leaving")
            break


def force_disarm() -> None:
    service_proxies.mavros_command(
        command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        param1=0,  # disarm
        param2=21196,  # force
    )


def navigate_wait(
    x: float,
    y: float,
    z: float,
    speed: float,
    frame_id: str,
    auto_arm: bool,
    timeout: float = 0,
) -> bool:
    """Navigate and, if given a timeout, return bool representing if timeout was reached prematurely. Default is false."""
    t0 = 0  # secs
    timeout_passed: bool = False
    if timeout > 0:  # seconds
        t0 = rospy.get_rostime().secs
    # begin navigating using px4's autopilot
    service_proxies.navigate(
        x=z, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm
    )
    # spin until we are within a certain threshold from target
    while not rospy.is_shutdown():
        # get current position relative to desired position
        telem = service_proxies.get_telemetry(frame_id="navigate_target")
        if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < action_movement_threshold:
            break
        # if timeout passes and we're not at desired point, terminate early
        if timeout > 0 and rospy.get_rostime().secs - t0 > timeout:
            timeout_passed = True
            break
        rospy.sleep(0.2)
    return timeout_passed


def episode_init_and_grab_state(gazebo_world_filepath: str) -> State:
    """Reset the drone's position and return the new state."""
    import rosnode

    if len(rosnode.get_node_names()) > 2:
        # kill simulation
        simulation_nodes.kill_clover_simulation()
    # start simulation
    simulation_nodes.launch_clover_simulation(gazebo_world_filepath)
    # await simulation to come online by reinitializing service proxies
    service_proxies.init()
    state_mutex.acquire()
    local_state = copy.deepcopy(state)
    state_mutex.release()
    return local_state


def episode_take_action(action: Action) -> Tuple[State, float, bool]:
    """Take the given action in the environment."""
    global state
    # coordinate systems in clover:
    # https://clover.coex.tech/en/frames.html#coordinate-systems-frames

    print("[episode_take_action]", action)
    # navigate a distance relative to the quadcopter's frame
    # The 'body' frame remains upright regardless of
    # the quadcopter's orientation.
    (x, y, z) = util.convert_spherical_to_cartesian(action.r, action.theta, action.phi)
    timeout_passed = navigate_wait(
        x=x,
        y=y,
        z=z,
        speed=1,
        frame_id="body",
        auto_arm=True,
        timeout=10,
    )
    state_mutex.acquire()
    local_state = copy.deepcopy(state)
    state_mutex.release()
    # pause physics to keep drone from drifting away
    reward = episode_calculate_reward_metric(local_state, timeout_passed)
    done = episode_calculate_if_done(local_state)
    return (local_state, reward, done)


def calibrate_accelerometers() -> None:
    # https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_CALIBRATION
    rospy.loginfo("[calibrate_accelerometers]")
    if not service_proxies.mavros_command(
        command=mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, param5=4
    ).success:
        return False

    calibrating = False
    while not rospy.is_shutdown():
        mavros_state = rospy.wait_for_message("mavros/state", mavros_msg.State)
        if (
            mavros_state.system_status == mavutil.mavlink.MAV_STATE_CALIBRATING
            or mavros_state.system_status == mavutil.mavlink.MAV_STATE_UNINIT
        ):
            calibrating = True
        elif (
            calibrating
            and mavros_state.system_status == mavutil.mavlink.MAV_STATE_STANDBY
        ):
            rospy.loginfo("Calibrating finished")
            return True


std_dev = 0.2
ou_noise = OUActionNoise(
    mean=np.zeros(num_actions), std_deviation=float(std_dev) * np.ones(num_actions)
)

actor_model_weights_filepath = "ddpg_actor.h5"
criticl_model_weights_filepath = "ddpg_critic.h5"

target_actor_weights_filepath = "ddpg_target_actor.h5"
target_critic_weights_filepath = "ddpg_target_critic.h5"

actor_model = get_actor()
critic_model = get_critic()

target_actor = get_actor()
target_critic = get_critic()

# Making the weights equal initially
target_actor.set_weights(actor_model.get_weights())
target_critic.set_weights(critic_model.get_weights())

# Save the weights before running so that
# we can load them in the main loop
actor_model.save(actor_model_weights_filepath)
critic_model.save(criticl_model_weights_filepath)

target_actor.save(target_actor_weights_filepath)
target_critic.save(target_critic_weights_filepath)

# Learning rate for actor-critic models
critic_lr = 0.002
actor_lr = 0.001

critic_optimizer = tf.keras.optimizers.Adam(critic_lr)
actor_optimizer = tf.keras.optimizers.Adam(actor_lr)

num_episodes = 10

# Discount factor for future rewards
gamma = 0.99
# Used to update target networks
tau = 0.005

# identifier for this program's execution
# used for saving files related to this execution
identifier = datetime.datetime.now().isoformat()

for gazebo_world_filepath in cirriculum_worlds:
    # load weights from saved location
    actor_model = tf.keras.models.load_model(actor_model_weights_filepath)
    critic_model = tf.keras.models.load_model(criticl_model_weights_filepath)
    target_actor = tf.keras.models.load_model(target_actor_weights_filepath)
    target_critic = tf.keras.models.load_model(target_critic_weights_filepath)

    buffer = Buffer(50000, 64)

    # To store reward history of each episode
    reward_history = {
        "episodic_reward": [],
        "average_reward": [],
        "world_file": [],
        "timestamp": [],
    }

    for ep in range(num_episodes):
        prev_state: State = episode_init_and_grab_state(gazebo_world_filepath)
        episodic_reward = 0

        num_actions_taken = 0
        num_actions_per_ep = 4

        while not rospy.is_shutdown() and num_actions_taken < num_actions_per_ep:
            # learning has completed or episode has restarted;
            # we can unpause physics engine
            service_proxies.unpause_physics()

            tf_prev_state = tf.expand_dims(
                tf.convert_to_tensor(util.dataclass_to_list(prev_state)), 0
            )

            action = policy(tf_prev_state, ou_noise)
            print("[TRACE] policy calculated")
            # Recieve state and reward from environment.
            local_state, reward, done = episode_take_action(action)
            print("[TRACE] action taken")
            num_actions_taken += 1
            # pause physics engine while learning is taking place
            service_proxies.pause_physics()

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

        reward_history["episodic_reward"].append(episodic_reward)
        # Mean of last 40 episodes
        avg_reward = np.mean(reward_history["episodic_reward"][-40:])
        reward_history["average_reward"].append(avg_reward)
        reward_history["world_file"].append(gazebo_world_filepath)
        reward_history["timestamp"].append(datetime.datetime.now().isoformat())

    # Save the weights
    actor_model.save_weights(actor_model_weights_filepath)
    critic_model.save_weights(criticl_model_weights_filepath)

    target_actor.save_weights(target_actor_weights_filepath)
    target_critic.save_weights(target_critic_weights_filepath)

    # save training metadata
    util.save_clover_train_metadata(reward_history, identifier)

rospy.signal_shutdown(f"Total episodes: {num_episodes}; Signaling shut down.")

local_position_thread.join()
velocity_thread.join()
rangefinder_thread.join()
