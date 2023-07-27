import pinocchio
import casadi as cs
from pinocchio import casadi as cpin


class Robot_dynamics:

    def __init__(self,):
        model = pinocchio.buildModelFromUrdf("/home/simulations/urdf/flywheel_pendulum.urdf")
 
        cmodel = cpin.Model(model)
        cdata = cmodel.createData()

        nq = model.nq
        nv = model.nv
        nx = nq + nv


        state = cs.SX.sym("x", nx, 1)
        cq = state[:nq]
        cv = state[nq:]
        tau = cs.SX.sym("tau", nv, 1)
        self.compute_acc = cpin.aba(cmodel, cdata, state[:nq], state[nq:], tau)
        self.compute_acc_f = cs.Function("cacc", [state, tau], [self.compute_acc])

    
    def forward_dynamics(self, state, tau):
        
        tau = cs.vertcat(0,tau)
        qdd = self.compute_acc_f(cs.vertcat([0, 0, 0], state), tau)

        xdot = cs.vertcat(state[1], qdd)

        return xdot