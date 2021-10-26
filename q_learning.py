import gym
import irb120_env
import pathlib
import tensorflow as tf
from tensorflow.python.ops.gen_math_ops import tanh_grad_eager_fallback

from tf_agents.agents.dqn import dqn_agent
from tf_agents.drivers import dynamic_step_driver
from tf_agents.environments import tf_py_environment
from tf_agents.eval import metric_utils
from tf_agents.metrics import tf_metrics
from tf_agents.networks import q_network
from tf_agents.replay_buffers import tf_uniform_replay_buffer
from tf_agents.trajectories import trajectory
from tf_agents.utils import common

# TODO Need to implement an agent

def main():
    env = gym.make("IRB120-v0")

    # Convert env numpy arrays to tensors for tensorflow 
    env = tf_py_environment.TFPyEnvironment(env)
    q_net = q_network.QNetwork(env.observation_spec(), env.action_spec)

    optimizer = tf.compat.v1.train.AdamOptimizer(learning_rate=.001)

    train_step_counter = tf.Variable(0)

    agent = dqn_agent(env.time_step_spec(),
                        env.action_spec(),
                        optimizer=optimizer,
                        td_errors_loss_fn=common.element_wise_squared_loss,
                        train_step_counter=train_step_counter)


    for i in range(10):
        observation = env.reset()
        for t in range(10000):
            action = env.action_space.sample()
            # print(env.prev_error)
            observation, reward, done, info = env.step(action)
            if done:
                print(f"Feached the target in {t} steps")
                break

    env.close()


if __name__ == "__main__":
    main()