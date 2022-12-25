import numpy as np

import sys
sys.path.append('/home/python_scripts/')
import euler_integration
from robot_dynamics import Robot_dynamics

import casadi as cs



class Adaptive_LQR:
    """This is a small class that computes an LQR control law based on a least square update"""


    def __init__(self, lin_state = None, lin_tau = None, horizon = None, dt = None):
        """
        Args:
            lin_state (np.array): linearization state
            lin_tau (np.array): linearization control inputs
            horizon (int): how much to look into the future for optimizing the gains 
            dt (int): desidered sampling time
        """
        if(horizon==None):
            self.horizon = 2000
        else:
            self.horizon = horizon
        self.dt = dt

        self.robot = Robot_dynamics()
        self.Q = np.identity(self.robot.state_dim)
        self.Q[0,0] = 10 #theta
        self.Q[1,1] = 0.2 #theta_dot
        self.Q[2,2] = 0.01 #phi_dot
        self.R = np.identity(self.robot.control_dim)*1

        self.lin_state = np.zeros(self.robot.state_dim)
        self.lin_tau = np.zeros(self.robot.control_dim)

        self.error_vec = np.array([])

        self.K = self.calculate_discrete_LQR_gain(self.lin_state, self.lin_tau)

        # adaptive law parameters -----------------------------------------------------
        self.gamma = 0.005
        self.adaptive_gains = np.array([[0]])


        # calculate error gradient for MIT rule ---------------------------------------
        previous_state = cs.SX.sym("previous_state", self.robot.state_dim, 1)
        state_des = cs.SX.sym("state_des", self.robot.state_dim, 1)
        state_meas = cs.SX.sym("state_meas", self.robot.state_dim, 1)
        adaptive_gains = cs.SX.sym("adaptive_gains", 1, 1)
        self.compute_error_f = self.compute_error(previous_state, state_des, state_meas, adaptive_gains)
        
        error_gradient = cs.jacobian(self.compute_error_f, adaptive_gains)
        self.error_gradient_f = cs.Function("error_gradient", [previous_state, state_des, state_meas, adaptive_gains], [error_gradient])


    def calculate_discrete_LQR_gain(self,lin_state, lin_tau):
        """Calculate by backward iterations the optimal LQR gains

        Args:
            lin_state (np.array): linearization state
            lin_tau (np.array): linearization control inputs

        Returns:
             K (np.array): optimal gains
        """
        P_next = np.identity(self.robot.state_dim)

        A = self.robot.A_f(lin_state, lin_tau)
        B = self.robot.B_f(lin_state, lin_tau)

        A_discrete = A*self.dt/10. + np.identity(self.robot.state_dim)
        B_discrete = B*self.dt/10.



        for i in range(0, self.horizon):
            Q_uu = self.R + B_discrete.T@P_next@B_discrete

            temp = (-np.linalg.pinv(Q_uu)@B_discrete.T@P_next@A_discrete)
            P_next = self.Q + A_discrete.T@P_next@A_discrete - temp.T@Q_uu@temp
            
            self.K = (np.linalg.pinv(self.R + B_discrete.T@P_next@B_discrete)@B_discrete.T@P_next@A_discrete)

        return self.K





    def compute_error(self, previous_state, state_des, state_meas, adaptive_gains):
        """Compute error signal
        Args:
            previous_state (np.array): state at timestep K-1
            state_des (np.array): state_des at timestep K-1
            state_meas (np.array): state measured at time K
            adaptive_gains (np.array)
        Returns:
            (np.array): error signal
        """

        control = self.K@(state_des - previous_state) # to add adaptive gain
        error = state_des - previous_state
        control += adaptive_gains[0][0]*error[1] 

        cs.reshape(control, 1, 1)
        
        next_state = self.robot.forward_dynamics(previous_state, control)
        qdd = next_state[1:3]
        state_pred = euler_integration.euler_integration_cs(previous_state, qdd, self.dt)

          
        #should be n_datapoints x num_features
        error_prediction = state_meas[1] - state_pred[1]

        return cs.reshape(error_prediction,1,1)
    

    
    def compute_adaptive_gains(self, previous_state, state_des, state_meas):
        """Compute a single adaptive gains update
        Args:
            previous_state (np.array): state at timestep K-1
            control (np.array): control at timestep K-1
            state_meas (np.array): state measured at time K  
        """

        error = self.compute_error(previous_state, state_des, state_meas, cs.reshape(self.adaptive_gains,1,1))
        error_grad = self.error_gradient_f(previous_state, state_des, state_meas, cs.reshape(self.adaptive_gains,1,1))
        error_np = np.array([error[0].__float__()])

        
        self.adaptive_gains = self.adaptive_gains - self.gamma*error_np.reshape(1,1)@error_grad




    


    def compute_control(self, state, state_des):
        """Compute feedforward and LQR control inputs
        Args:
            state (np.array): actual robot state
            state_des (np.array): desired robot state

        Returns:
            (np.array): optimized control inputs

        """
        
        u_ff = 0
        control = u_ff + self.K@(state_des - state)
        error = state_des - state
        control += self.adaptive_gains[0][0]*error[1]

        print("lqr gains: ", self.K)
        print("adaptive gain: ", self.adaptive_gains)

        return control




if __name__=="__main__":
    control = Adaptive_LQR(dt=0.01, horizon=2000)
    
    x1 = np.array([0, 0, 0])
    x1_des = np.array([0, 1, 1])
    u1 = np.array([0.1])
    y1_meas = np.array([0, 0, 0])




    control.compute_adaptive_gains(x1, x1_des, y1_meas)
    control.compute_adaptive_gains(x1, x1_des, y1_meas)

    #control.recursive_least_square(x1, u1, y1_meas)

