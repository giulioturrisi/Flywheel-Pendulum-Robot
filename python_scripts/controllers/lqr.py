import numpy as np

import sys
sys.path.append('/home/python_scripts/')
from robot_dynamics import Robot_dynamics
import euler_integration

class LQR:
    """This is a small class that computes a simple LQR control law"""


    def __init__(self, lin_state = None, lin_tau = None, horizon = None, dt = None):
        """
        Args:
            lin_state (np.array): linearization state
            lin_tau (np.array): linearization control inputs
            horizon (int): how much to look into the future for optimizing the gains 
            dt (int): desidered sampling time
        """
        self.state_dim = 3
        
        self.lin_state = np.zeros(self.state_dim)
        self.lin_tau = np.zeros(1)
        self.horizon = 2000
        self.dt = dt

        self.robot = Robot_dynamics()

        self.Q = np.identity(self.state_dim)
        self.Q[0,0] = 10 #theta
        self.Q[1,1] = 0.2 #theta_dot
        self.Q[2,2] = 0.01 #phi_dot


        self.R = np.identity(1)*1
        
        self.K = self.calculate_discrete_LQR_gain(self.lin_state, self.lin_tau)



    def calculate_discrete_LQR_gain(self,lin_state, lin_tau):
        """Calculate by backward iterations the optimal LQR gains

        Args:
            lin_state (np.array): linearization state
            lin_tau (np.array): linearization control inputs

        Returns:
             K (np.array): optimal gains
        """
        P_next = np.identity(self.state_dim)

        A = self.robot.A_f(lin_state, lin_tau)
        B = self.robot.B_f(lin_state, lin_tau)

        A_discrete = A*self.dt + np.identity(self.state_dim)
        B_discrete = B*self.dt



        for i in range(0, self.horizon):
            Q_uu = self.R + B_discrete.T@P_next@B_discrete

            temp = (-np.linalg.pinv(Q_uu)@B_discrete.T@P_next@A_discrete)
            P_next = self.Q + A_discrete.T@P_next@A_discrete - temp.T@Q_uu@temp
            
        self.K = (np.linalg.pinv(self.R + B_discrete.T@P_next@B_discrete)@B_discrete.T@P_next@A_discrete)

        return self.K



    def compute_control(self, state, state_des):
        """Compute feedforward and LQR control inputs

        Args:
            state (np.array): actual robot state
            state_des (np.array): desired robot state

        Returns:
            (np.array): optimized control inputs

        """

        return self.K@(state_des - state)


if __name__=="__main__":
    controller = LQR(dt = 0.001)
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

        
        
