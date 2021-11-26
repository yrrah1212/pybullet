import gym

import numpy as np
from numpy.random import default_rng

import pybullet as p
import pybullet_data

from irb120_env.resources.arm import Arm

class IRB120ENV(gym.Env):
    metadata = {'render.modes': ['human']}  
  
    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            # Action space bounded by joint limits
            low=np.array([-2.87979, -1.91986, -1.91986, -2.79253, -2.094395, -6.98132]),
            high=np.array([2.87979, 1.91986, 1.22173, 2.79253, 2.094395, 6.98132])
        )

        self.observation_space = gym.spaces.box.Box(
            # Position and orientation of end effector. x, y, z, quaternion values
            low=np.array([-5, -5, -5, -1, -1, -1, -1]),
            high=np.array([5, 5, 5, 1, 1, 1, 1])
        )
        self.seed()

        # Connect to the pybullet sim
        self.sim = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.world_plane = p.loadURDF("plane.urdf")

        p.setGravity(0,0,-9.8)

        self.arm = None
        self.goal = None
        self.done = None
        self.prev_error = None
        self.step_counter = 0

        self.reset()


    def step(self, action):
        done_cause = ''

        # Apply the action to the arm and step simulation
        self.arm.apply_action(action)
        p.stepSimulation()
        
        # Get observation about the arm now
        arm_state = self.arm.get_observations()

        # Calculate the new error. L2 distance between goal vectors
        error = np.sqrt(np.sum([(self.goal[i] - arm_state[i])**2 for i in range(7)]))

        # Reward. Difference between previous error and current error if there were no collisions
        collisions = p.getContactPoints()
        if len(collisions) > 0:
            reward = -100
            # self.done = True
        # else:
        #     reward = max(self.prev_error - error, 0)
        reward = 1 / abs(error)**2

        # Update previous error
        self.prev_error = error

        # Increase the step counter
        self.step_counter += 1

        # If the step counter goes over this many steps then stop
        if self.step_counter > 500:
            done_cause = 'step_counter '
            self.done = True

        # Check if the process is done
        # TODO determine if this reward is appropriate for solving the problem
        if error < .001:
            done_cause += 'error'
            reward = 2000
            self.done = True

        # Return the observation, reward, and done state
        return np.subtract(self.goal, arm_state), reward, self.done, dict({'done_cause':done_cause})



    def reset(self):
        self.done = False
        self.step_counter = 0

        p.resetSimulation()
        self.world_plane = p.loadURDF("plane.urdf")
        p.setGravity(0,0,-9.8)

        self.arm = Arm()

        # Generate a goal position and orientation for the arm
        # Limits based on arm contraints
        # https://new.abb.com/products/robotics/industrial-robots/irb-120/irb-120-data
        # x_max = .4
        # x = (default_rng().random() * 2 * x_max) - x_max
        # y_max = np.sqrt(x_max - x**2)
        # y = (default_rng().random() * 2 * y_max) - y_max
        # z_max = np.sqrt(.8**2 - x**2 - y**2)
        # z = (default_rng().random() * 2 * z_max) - z_max
        # goal_d = [x, y, z]

        # goal_q = 2*default_rng().random(4)-1
        # goal_q /= np.linalg.norm(goal_q)

        goal_d = [.5, .5, .5]
        goal_q = [1, 0, 0, 0]

        self.goal = np.concatenate([goal_d, goal_q])

        # Add goal position to the sim. Sphere with radius=.1
        goal_collision = p.createCollisionShape(p.GEOM_SPHERE, .05)
        goal_visual = p.createVisualShape(p.GEOM_SPHERE, .05, rgbaColor=[0,1,0,1])
        p.createMultiBody(baseCollisionShapeIndex=goal_collision, 
                            baseVisualShapeIndex=goal_visual,
                            basePosition=goal_d)

        # Get observation for the current arm state
        arm_state = self.arm.get_observations()

        # Set the first prev_error based on the starting error
        error = np.sqrt(np.sum([(arm_state[i] - self.goal[i])**2 for i in range(7)]))
        self.prev_error = error

        return np.subtract(self.goal, arm_state)


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


    # Set the pose of the camera for rendering
    def set_cam_pose(self, cam_pos):
        self.cam_pos = cam_pos


    def close(self):
        p.disconnect()   
    

    def seed(self, seed=None): 
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]