import numpy as np
import casadi as cs
import time

import sys
sys.path.append('/home/python_scripts/')
from pinocchio_dynamics import Robot_dynamics
import euler_integration


class Casadi_NMPC:
    """This is a small class that computes a NMPC control law"""


    def __init__(self, N = 10, dt = 0.005):
        """
        Args:
        """
        self.state_dim = 3
        self.control_dim = 1
        

        self.N = 20
        self.dt = dt

        self.Q = np.identity(self.state_dim)
        self.Q[0,0] = 10 #theta
        self.Q[1,1] = 0.02 #theta_dot
        self.Q[2,2] = 0.001 #phi_dot

        self.R = np.identity(1)*0.01


        self.robot = Robot_dynamics()
        self.initialize_casadi()



    def initialize_casadi(self):
        """Initialize the casadi optimal control problem
        """

        # Casadi problem formulation ---------------------------------------
        self.opti = cs.Opti() # Optimization problem
        
        # Decision variables ---------------------------------------
        self.X_casadi = self.opti.variable(self.state_dim, self.N+1) # state trajectory
        self.theta_casadi   = self.X_casadi[0,:]
        self.theta_dot_casadi   = self.X_casadi[1,:]
        self.phi_dot_casadi   = self.X_casadi[2,:]

        self.U_casadi = self.opti.variable(self.control_dim, self.N) # control trajectory

        # Initial State Constraint -----------------------------------
        self.theta_0 = self.opti.parameter()
        self.theta_dot_0 = self.opti.parameter()
        self.phi_dot_0 = self.opti.parameter()
        self.opti.subject_to(self.theta_casadi[0]==self.theta_0)
        self.opti.subject_to(self.theta_dot_casadi[0]==self.theta_dot_0)
        self.opti.subject_to(self.phi_dot_casadi[0]==self.phi_dot_0) 

        # State Constraints and Cost Function -------------------------
        self.set_kinematics()
        self.set_cost_function()
        
        # Solver parameters ---------------------------------------
        p_opts = dict(print_time=False, verbose=False) 
        s_opts = dict(print_level=0)
        self.opti.solver("ipopt",p_opts,s_opts) # set numerical backend


    def set_kinematics(self):
        """Setting the kinematics constraints
        """

        # Kynematic Constraints ---------------------------------------
        f = lambda x,u: vertcat(x[1],u-x[1]) # dx/dt = f(x,u)
        for k in range(self.N): # loop over control intervals

            # integration
            x_dot = self.robot.forward_dynamics(self.X_casadi[:,k], self.U_casadi[:,k])
            qdd = x_dot[1:3]
            #state = euler_integration.euler_integration_cs(self.X_casadi[:,k], qdd, self.dt).reshape(self.state_dim,)
            
            theta = self.X_casadi[0,k]
            theta_dot = self.X_casadi[1,k]
            phi_dot = self.X_casadi[2,k]
            
            theta = theta + theta_dot*self.dt
            theta_dot = theta_dot + qdd[0]*self.dt

            #phi = phi + phi_dot*dt
            phi_dot = phi_dot + qdd[1]*self.dt

            next_theta = theta
            next_theta_dot = theta_dot
            next_phi_dot = phi_dot

            # close the gaps
            self.opti.subject_to(self.X_casadi[0,k+1]==next_theta)
            self.opti.subject_to(self.X_casadi[1,k+1]==next_theta_dot) 
            self.opti.subject_to(self.X_casadi[2,k+1]==next_phi_dot) 



    def set_cost_function(self):
        """Setting the cost function
        """

        # Parametric Cost Function -------------------------------------
        state_error = 0
        input_use = 0
        
        self.theta_des = self.opti.parameter()
        self.theta_dot_des = self.opti.parameter()
        self.phi_dot_des = self.opti.parameter()

        for k in range(1, self.N):

            state_error += self.Q[0,0]*(self.theta_casadi[k] - self.theta_des)@(self.theta_casadi[k] - self.theta_des).T
            state_error += self.Q[1,1]*(self.theta_dot_casadi[k] - self.theta_dot_des)@(self.theta_dot_casadi[k] - self.theta_dot_des).T
            state_error += self.Q[2,2]*(self.phi_dot_casadi[k] - self.phi_dot_des)@(self.phi_dot_casadi[k] - self.phi_dot_des).T

            input_use += self.R*(self.U_casadi[0,k])@(self.U_casadi[0,k]).T 


        
        # Last step N horizon -----------------------------------------------------------------------
            state_error += self.Q[0,0]*(self.theta_casadi[self.N] - self.theta_des)@(self.theta_casadi[self.N] - self.theta_des).T
            state_error += self.Q[1,1]*(self.theta_dot_casadi[self.N] - self.theta_dot_des)@(self.theta_dot_casadi[self.N] - self.theta_dot_des).T
            state_error += self.Q[2,2]*(self.phi_dot_casadi[self.N] - self.phi_dot_des)@(self.phi_dot_casadi[self.N] - self.phi_dot_des).T

        
        self.opti.minimize(state_error + input_use)
     


    def compute_control(self, state, state_des):
        """
        """


        # Setting Initial State ---------------------------------------
        start_time = time.time()
        self.opti.set_value(self.theta_0, state[0])
        self.opti.set_value(self.theta_dot_0, state[1])
        self.opti.set_value(self.phi_dot_0, state[2])

        # Setting Reference ------------------------------------------
        self.opti.set_value(self.theta_des, state_des[0])
        self.opti.set_value(self.theta_dot_des, state_des[1])
        self.opti.set_value(self.phi_dot_des, state_des[2])
           
        # Compute solution ---------------------------------------
        start_time = time.time()
        sol = self.opti.solve()
        print("Solving time: ", time.time()-start_time)
        return sol.value(self.U_casadi)[0]


if __name__=="__main__":
    controller = Casadi_NMPC(dt = 0.005)
    state = np.zeros(controller.state_dim)
    state[0] = 0.5 
    state_des = np.zeros(controller.state_dim)
    
    
    for j in range(0,500):
        print("################")
        tau = controller.compute_control(state, state_des=state_des)
        tau = tau 
        print("tau", tau)
        print("state: ", state)

        x_dot = controller.robot.forward_dynamics(state, tau)
        
        qdd = x_dot[1:3]
        state = euler_integration.euler_integration(state, qdd, controller.dt).reshape(controller.state_dim,)

        
        
