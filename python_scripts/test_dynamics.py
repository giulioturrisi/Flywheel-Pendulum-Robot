import numpy as np

import sys
sys.path.append('/home/python_scripts/')
import euler_integration
from robot_dynamics import Robot_dynamics
from pinocchio_dynamics import Pinocchio_dynamics

import matplotlib.pyplot as plt # type: ignore

import time

import copy

import casadi as cs




state_dim = 3
control_dim = 1
state = np.zeros(state_dim)
state.reshape(state_dim,1) 
control = np.zeros(control_dim)+0.1
control = control.reshape(control_dim,)

robot1 = Robot_dynamics()
robot2 = Pinocchio_dynamics()


print("robot1.forward_dynamics(state,control)", robot1.forward_dynamics(state, control))


#state_dim = 6
#state = np.zeros(state_dim)
#state.reshape(state_dim,1) 
print("robot2.forward_dynamics(state,control)", robot2.forward_dynamics(state, control))
