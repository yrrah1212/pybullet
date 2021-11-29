from urllib.parse import DefragResult
from numpy.core.fromnumeric import shape
import gym

import numpy as np
from numpy.random import default_rng

import pybullet as p
import pybullet_data

from irb120_env_simple.resources.arm import Arm

class IRB120ENV_simple(gym.Env):
    metadata = {'render.modes': ['human']}  
  
    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            # Action space for theta 1
            low=np.array([-2.87979]),
            high=np.array([2.87979])
        )

        self.observation_space = gym.spaces.box.Box(
            low=np.array([-5, -5, -5, -5, -5, -5]),
            high=np.array([5, 5, 5, 5, 5, 5])
        )
        
        self.seed()

        # Connect to the pybullet sim
        self.sim = p.connect(p.DIRECT)

        self.arm = None
        self.goal = None
        self.done = False
        self.prev_error = 0
        self.step_counter = 0

        # Max number of steps per iteration
        self.max_steps = 50

        self.reset()


    def step(self, action):
        termination = ''
        # Apply the action to the arm and step simulation
        self.arm.apply_action(action)
        # p.stepSimulation()
        
        # Get observation about the arm now
        arm_state = self.arm.get_observations()

        # Calculate the new error. L2 distance between goal vectors
        error = np.subtract(self.goal, arm_state)

        # Reward is based on how close to the target the arm is, not how much closer it has moved towards the goal
        # Try statement to avoid issues with dividing by zero
        reward = -1*np.linalg.norm(error)

        self.prev_error = error         

        # Increase the step counter
        self.step_counter += 1

        # If the step counter goes over this many steps then stop
        if self.step_counter > self.max_steps:
            termination = 'step counter'
            self.done = True

        # Check if the process is done
        if np.abs(error) <= .1:
            self.done = True
            termination = 'min error'

        return np.concatenate((arm_state, error)), reward, self.done, dict({'termination':termination})


    def reset(self):
        self.done = False
        self.step_counter = 0

        p.resetSimulation()

        self.arm = Arm()

        angle = default_rng().random()*2*2.8 - 2.8
        # angle = -1
        x = .34 * np.cos(angle)
        y = .34 * np.sin(angle)
        z = -.084
        goal_d = [x,y,z]

        # self.goal = goal_d
        self.goal = goal_d

        # Add goal position to the sim. Sphere with radius=.1
        goal_visual = p.createVisualShape(p.GEOM_SPHERE, .05, rgbaColor=[0,1,0,1])
        p.createMultiBody(  baseVisualShapeIndex=goal_visual,
                            basePosition=goal_d)

        # Get observation for the current arm state
        arm_state = self.arm.get_observations()

        # Set the first prev_error based on the starting error
        error = np.subtract(self.goal, arm_state)

        self.prev_error = error

        # returns error as the current state so the state is based on the goal
        return np.concatenate((arm_state, error))


    def render(self, mode=None, args=None):
        # Location of the target (the base of the arm)
        cam_pos = [1,-1,1.5]
        target_pos = [0,0,.25]
        up_vector = [0,0,1]

        # Parameters for the rendered image
        fov = 60
        img_width = 640
        img_height = 480
        aspect = img_width / img_height

        # View and projection matrices from pybullet
        view_matrix = p.computeViewMatrix(cam_pos, target_pos, up_vector)
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, .01, 100)

        # The image from pybullet
        img_array = p.getCameraImage(img_width, img_height, view_matrix, projection_matrix)
        w = img_array[0]
        h = img_array[1]
        rgb = img_array[2]
        # Convert the image
        np_img = np.reshape(rgb, (h,w,4))
        # np_img = np_img * (1./255.)
        return np_img

    def close(self):
        p.disconnect()   
    

    def seed(self, seed=None): 
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]