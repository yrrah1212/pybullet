import gym
import numpy as np
import pybullet as p
import pybullet_data
from irb120_env.resources import Arm

class IRB120ENV(gym.Env):
    metadata = {'render.modes': ['human']}  
  
    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            # Action space bounded by joint limits
            low=np.array([-2.87979, -1.91986, -1.91986, -2.79253, -2.094395, -6.98132]),
            high=np.array([2.87979, 1.91986, 1.22173, 2.79253, 2.094395, -6.98132])
        )

        self.observation_space = gym.spaces.box.Box(
            # Position and orientation of end effector. x, y, z, quaternion values
            low=np.array([-5, -5, -5, -1, -1, -1, -1]),
            high=np.array([5, 5, 5, 5, 1, 1, 1, 1])
        )
        self.seed()

        self.sim = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-9.8)
        planeId = p.loadURDF("plane.urdf")

        self.arm = None
        self.goal = None
        self.done = None
        self.prev_error = None
        self.visualize = False

        self.reset()


    def step(self, action):

        # Apply the action to the arm and step simulation
        self.arm.apply_action(action)
        p.stepSimulation()
        
        # Get observation about the arm now
        arm_state = self.arm.get_observations()

        # TODO calculate the new error. Probably L2 distance between end effector and destination 
        #       but could also incorporate rotation error

        # TODO calculate reward

        # TODO update previous error

        # TODO check if the process is done

        # TODO return the observation, reward, and done state

    def reset(self):
        p.resetSimulation()
        p.setGravity(0,0,-9.8)

        self.arm = Arm()

        # TODO generate a goal position and orientation for the arm

        # TODO get observation for the current arm state

        # TODO return current observation

    def render(self):
        pass

    def close(self):
        p.disconnect()   
    
    def seed(self, seed=None): 
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]