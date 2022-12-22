import numpy as np

def euler_integration(state, qdd, dt):
    """Small function that compute a simple euler integration

    Args:
        state (np.array): state of the robot at tike K
        qdd (np.array): instantaneous acceleration of the system
        dt (float): sampling time

    Returns:
        (np.array): new state of the robot at time K+1
    """

    theta = state[0] 
    theta_dot = state[1]
    
    phi_dot = state[2]
    
    theta = theta + theta_dot*dt
    theta_dot = theta_dot + qdd[0]*dt

    #phi = phi + phi_dot*dt
    phi_dot = phi_dot + qdd[1]*dt

    state = np.array([theta, theta_dot.__float__(), phi_dot.__float__()])
    state = state.reshape(3,)

    return state