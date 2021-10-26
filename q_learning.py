import gym
import irb120_env
import pathlib
import tensorflow as tf
from tensorflow.python.ops.gen_math_ops import tanh_grad_eager_fallback

from tf_agents.agents.dqn import dqn_agent
from tf_agents.drivers import py_driver
from tf_agents.environments import suite_gym
from tf_agents.environments import tf_py_environment
from tf_agents.eval import metric_utils
from tf_agents.metrics import tf_metrics
from tf_agents.networks import sequential
from tf_agents.policies import py_tf_eager_policy
from tf_agents.policies import random_tf_policy
from tf_agents.replay_buffers import reverb_replay_buffer
from tf_agents.replay_buffers import reverb_utils
from tf_agents.trajectories import trajectory
from tf_agents.specs import tensor_spec
from tf_agents.utils import common

# TODO Need to implement an agent

def main():
    train_env = suite_gym.load("IRB120-v0")
    test_env = suite_gym.load("IRB120-v0")

    # Convert env numpy arrays to tensors for tensorflow 
    train_env = tf_py_environment.TFPyEnvironment(train_env)
    test_env = tf_py_environment.TFPyEnvironment(test_env)

    


if __name__ == "__main__":
    main()