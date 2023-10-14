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

num_inputs = 9
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

optimizer = keras.optimizers.Adam(learning_rate = 0.01)
huber_loss = keras.losses.Huber()
action_probs_history = []
critic_value_history = []
rewards_history = []
running_reward = 0
episode_count = 0

while True:
    reset_world()

    episode_reward = 0
    with tf.GradientTape() as tape:
        for timestamp in range(1, max_steps_per_episode):
            # get current state
            telemetry_info = get_telemetry()
            state = tf.convert_to_tensor([
                telemetry_info.x,
                telemetry_info.y,
                telemetry_info.z,
                telemetry_info.vx,
                telemetry_info.vy,
                telemetry_info.vz,
                telemetry_info.roll,
                telemetry_info.pitch,
                telemetry_info.yaw
            ])
            state = tf.expand_dims(state, 0)

            # predict action probabilities and estimated future rewards
            # from environment state
            action_probs, critic_value = model(state)
            critic_value_history.append(critic_value[0, 0])

            # sample action from probability distribution
            # FIXME: how to choose action?
            # we have:
            #   thrust
            #   pitch
            #   roll
            #   yaw
            action = np.random.choice(num_actions, p=np.squeeze(action_probs))
            action_probs_history.append(tf.math.log(action_probs[0, action]))

            # apply the sampled action in our environment
            # FIXME: change attitude
            set_attitude(
                pitch=1.0,
                roll=1.0, 
                thrust=1.0,
                yaw=1.0
            )

            telemetry_info = get_telemetry()
            # get current distance from (0,0,1)
            desired_pos = [0,0,1]
            current_pos = [telemetry_info.x, telemetry_info.y, telemetry_info.z]
            get_magnitude = lambda x: np.linalg.norm(x)
            distance_from_desired_pos = get_magnitude(np.subtract(desired_pos, current_pos))
            # reward is negative of the distance from the desired position
            reward = -distance_from_desired_pos
            episode_reward += reward

            # NOTE: have done metric?
            done = False

            if done:
                break

        
        running_reward = 0.05 * episode_reward + (1 - 0.05) * running_reward

        # calculate expected value from rewards
        # - at each timestep what was the total reward received after that timestep
        # - rewards in the past are discounted by multiplying them with gamma
        # - these are the labels for our critic
        returns = []
        discounted_sum = 0
        for r in rewards_history[::-1]:
            discounted_sum = r + gamma * discounted_sum
            returns.insert(0, discounted_sum)

        # normalize
        returns = np.array(returns)
        returns = (returns - np.mean(returns)) / (np.std(returns) + epsilon)
        returns = returns.tolist()

        # calculating loss values to update our network
        history = zip(action_probs_history, critic_value_history, returns)
        actor_losses = []
        critic_losses = []
        for log_prob, value, ret in history:
            diff = ret - value
            actor_losses.append(-log_prob + diff)
            critic_losses.append(
                huber_loss(tf.expand_dims(value, 0), tf.expand_dims(ret, 0))
            )
        # backpropagation
        loss_value = sum(actor_losses) + sum(critic_losses)
        grads = tape.gradient(loss_value, model.trainable_variables)
        optimizer.apply_gradients(zip(grads, model.trainable_variables))

        # clear the loss and reward history
        action_probs_history.clear()
        critic_value_history.clear()
        rewards_history.clear()

    # log details
    episode_count += 1
    if episode_count % 10 == 0:
        template = "running reward: {:.2f} at episode {}"
        print(template.format(running_reward, episode_count))

    if running_reward > 195: # condition to consider the task solved
        print("Solved at episode {}!".format(episode_count))
        break

            

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
