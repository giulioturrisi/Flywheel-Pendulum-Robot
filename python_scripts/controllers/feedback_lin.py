import numpy as np
import math

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/../')

import euler_integration
from robot_dynamics import Robot_dynamics



class Feedback_Lin:
    """This is a small class that computes a controller based on feedback linearization and a simple PID"""


    def __init__(self, k_p, k_d, k_i):
        """
        Args:
            k_d (float): linear velocity gain
            k_p (float): position gain
            k_i (float): angular position integral gain
        """
        self.state_dim = 3
        self.k_i = k_i 
        self.k_d = k_d
        self.k_p = k_p

        self.error_i = 0

        self.robot = Robot_dynamics()


    def compute_control(self, state, state_des):
        """Compute Feedback Lin control inputs

        Args:
            state (np.array): actual robot state
            state_des (np.array): desired robot state

        Returns:
            (np.array): control inputs

        """
        error_p = state_des[0] - state[0]
        error_d = state_des[1] - state[1]
        self.error_i += error_p

        tau = -(self.robot.d22/self.robot.det_D)*(-self.robot.m_bar*self.robot.g*np.sin(state[0]))/(self.robot.d12/self.robot.det_D)
        
        tau += -(self.k_p*(error_p) + self.k_d*(error_d) + self.k_i*(self.error_i))/(self.robot.d12/self.robot.det_D) 

        
        return tau


if __name__=="__main__":
    controller = Feedback_Lin(k_p = 30, k_d = 5, k_i = 0.001)
    state = np.zeros(controller.state_dim)
    state[0] = 1.8
    state[1] = 1.5 
    state_des = np.zeros(controller.state_dim)
    
    dt = 0.001
    for j in range(0,500):
        print("################")
        tau = controller.compute_control(state, state_des=state_des)
        tau = tau 
        print("tau", tau)
        print("state: ", state)

        x_dot = controller.robot.forward_dynamics(state, tau)
        qdd = x_dot[1:3]
        print("qdd: ", qdd)

        state = euler_integration.euler_integration(state, qdd, dt).reshape(controller.state_dim,)

        
        
