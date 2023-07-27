from acados_template import AcadosModel
from casadi import SX, vertcat

import sys
sys.path.append('/home/python_scripts/')
#from robot_dynamics import Robot_dynamics
from pinocchio_dynamics import Robot_dynamics

# Reference for model equations:
# http://users.isr.ist.utl.pt/~jag/publications/08-JETC-RCarona-vcontrol.pdf

def export_robot_model() -> AcadosModel:
    model_name = "flywheel_robot"

    # set up states & controls
    theta = SX.sym("theta")
    theta_d = SX.sym("theta_d")
    phi_d = SX.sym("phi_d")
    

    x = vertcat(theta, theta_d, phi_d)

    u = SX.sym("tau")


    # xdot
    theta_dot = SX.sym("x_dot")
    theta_ddot = SX.sym("theta_ddot")
    phi_ddot = SX.sym("phi_ddot")


    xdot = vertcat(theta_dot, theta_ddot, phi_ddot)

    # algebraic variables
    # z = None

    # parameters
    p = []

    # dynamics
    robot = Robot_dynamics()
    print("x", x.shape)
    print("u", u.shape)
    f_expl = robot.forward_dynamics(x, u)

    print("f_expl", f_expl)

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model