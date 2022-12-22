import numpy as np
import casadi as cs



class Robot_dynamics:
    """This class computes the forward dynamics, the linearization matrices, and other minor quantities
       based on the model for the two wheeled inverted pendulum 
    """

    def __init__(self,):
        """
        Args: Nothing!
        """
        self.state_dim = 3
        self.control_dim = 1
        # Robot parameters -----------------------------------------
        self.m_pendulum = 0.3046 #mass pendulum - kg
        self.m_flywheel = 0.28 #mass flywheel - kg
        self.I_pendulum = 0.043 #inertia pend
        self.I_flywheel = 0.005 #inertia flywheel
        self.l_p = 0.16 #distance from fulcrum pendulum to flywheel axis
        self.l_c = 0.08 #distance from fulcrum pendulum to center of mass pendulum

        self.g = 9.81 #gravity
        
        #self.alpha_1 = self.m_flywheel*self.l_p*self.l_p + self.I_pendulum
        #self.alpha_2 = (self.m_pendulum*self.l_c + self.m_flywheel*self.l_p)*self.g
        self.d11 = self.m_pendulum*self.l_c*self.l_c + self.m_flywheel*self.l_p*self.l_p + self.I_flywheel + self.I_pendulum
        self.d12 = self.I_flywheel
        self.d21 = self.d12
        self.d22 = self.d12

        self.det_D = self.d11*self.d22 - self.d12*self.d22

        self.m_bar = self.m_pendulum*self.l_c + self.m_flywheel*self.l_p

        self.friction_pendulum = 1.000e-03 
        self.friction_flywheel = 2.575e-03 

        
        # Precompute dynamics and linearization func. --------------
        state = cs.SX.sym("state", 3, 1)
        tau = cs.SX.sym("tau", 1, 1)
        
        forward_dynamics_f = self.forward_dynamics(state, tau)
        self.fd = cs.Function("fd", [state, tau], [forward_dynamics_f])

        self.A_f = self.get_A_f_matrix()
        self.B_f = self.get_B_f_matrix()



    def forward_dynamics(self, state, tau):
        """Compute forward dynamics
        Args:
            state (np.array): state of the robot at time K-1
            tau (np.array): control applied to the robot at time K-1
        Returns:
            (np.array): instantaneous acceleration of the system and velocity
        """
        #angle_pend, angle_flywheel, angle_dot_pendulun, angle_dot_flywheel
        theta = state[0] 
        #phi = state[1]
        theta_dot = state[1]
        phi_dot = state[2]
         
        # based on "Nonlinear control of the Reaction Wheel Pendulum", Spong et al.
        qd = state[1]
        theta_ddot = -(self.d22/self.det_D)*(-self.m_bar*self.g*cs.np.sin(theta)) - (self.d12/self.det_D)*tau
        phi_ddot =  (self.d21/self.det_D)*(-self.m_bar*self.g*cs.np.sin(theta)) + (self.d11/self.det_D)*tau
        
        qdd = cs.vertcat(theta_ddot,phi_ddot)
        return cs.vertcat(qd,qdd)



    def get_A_f_matrix(self, ):
        """Compute linearized A matrix function
            
        Returns:
            (cs.Function): function that can be used to calculate the A matrix
        """
        state_sym = cs.SX.sym("state", 3, 1)
        tau_sym = cs.SX.sym("tau", 1, 1)
        forward_dynamics_f = self.forward_dynamics(state_sym, tau_sym)

        A = cs.jacobian(forward_dynamics_f, state_sym)
        A_f = cs.Function("A", [state_sym, tau_sym], [A])
        return A_f



    def get_B_f_matrix(self, ):
        """Compute linearized B matrix function
            
        Returns:
            (cs.Function): function that can be used to calculate the B matrix
        """
        state_sym = cs.SX.sym("state", 3, 1)
        tau_sym = cs.SX.sym("tau", 1, 1)
        forward_dynamics_f = self.forward_dynamics(state_sym, tau_sym)

        B = cs.jacobian(forward_dynamics_f, tau_sym)
        B_f = cs.Function("B", [state_sym, tau_sym], [B])
        return B_f


    


if __name__=="__main__":
    Robot_dynamics()