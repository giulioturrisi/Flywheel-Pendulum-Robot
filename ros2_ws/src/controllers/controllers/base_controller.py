import rclpy # type: ignore 
from rclpy.node import Node # type: ignore

from std_msgs.msg import String, Bool, Float64, Float64MultiArray # type: ignore
from tf2_msgs.msg import TFMessage # type: ignore
from geometry_msgs.msg import PoseStamped, Twist, Vector3 # type: ignore
import tf_transformations # type: ignore

import copy
import matplotlib.pyplot as plt # type: ignore
import math
import time
import numpy as np # type: ignore
import sys
np.set_printoptions(threshold=sys.maxsize)
sys.path.append('/home/python_scripts/')
from robot_dynamics import Robot_dynamics

class Base_Controller(Node):
    def __init__(self, name):
        super().__init__(name)

        self.state_arrived = False
        self.dt = 0.01

        self.robot = Robot_dynamics()

        self.old_state_robot = np.zeros(self.robot.state_dim)

        self.state_robot = np.zeros(self.robot.state_dim)
        self.state_d = np.zeros(self.robot.state_dim)

        
        self.subscription_state = self.create_subscription(Vector3,'state',self.state_callback,1)

        self.publisher_command = self.create_publisher(Float64,"motor_cmd", 1);

 
        # Sincronization with simulation ---------------------------------------
        self.enableSyncMode = Bool();
        self.enableSyncMode.data = True;
        self.publisher_enableSyncMode =self.create_publisher(Bool,"enableSyncMode", 1);
        self.publisher_enableSyncMode.publish(self.enableSyncMode)

        self.triggerNextStep = Bool();
        self.triggerNextStep.data = True;
        self.publisher_triggerNextStep = self.create_publisher(Bool,"triggerNextStep", 1);

        self.simStep_done = True
        self.subscription_simStep = self.create_subscription(Bool,'simulationStepDone',self.simStep_callback,1)


    def publish_command(self, torque):

        commanded_torque = Float64()
        commanded_torque.data = np.float(torque)
        self.publisher_command.publish(commanded_torque)


    def state_callback(self, msg):
        self.old_state_robot = copy.deepcopy(self.state_robot)
        self.state_robot[0] = msg.x
        self.state_robot[1] = msg.y
        self.state_robot[2] = msg.z

        self.state_arrived = True


    def simStep_callback(self,msg):
        self.simStep_done = True

    def get_theta_d_callback(self, msg):
        self.state_d[0] = msg.data


    def triggerNextStep_Sim(self,):
        self.simStep_done = False
        self.publisher_triggerNextStep.publish(self.triggerNextStep)



def main(args=None):
    rclpy.init(args=args)

    controller_node = Base_Controller()

    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()