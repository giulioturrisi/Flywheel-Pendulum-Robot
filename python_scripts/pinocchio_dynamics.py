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
        self.cacc = cpin.aba(cmodel, cdata, state[:nq], state[nq:], tau)
        self.fd = cs.Function("cacc", [state, tau], [self.cacc])

    
    def forward_dynamics(self, state, tau):
        state = cs.vertcat([0, 0, 0], state)
        tau = cs.vertcat(0,tau)

        return cs.vertcat(state[1], self.fd(state, tau))