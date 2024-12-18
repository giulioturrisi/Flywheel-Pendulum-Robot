import mujoco
import mujoco.viewer

import numpy as np

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))


class MujocoSimulation:
    def __init__(self, ):
        self.scene_file_path = dir_path + "/flywheel_pendulum/scene.xml"
        self.mjModel = mujoco.MjModel.from_xml_path(str(self.scene_file_path))
        self.mjData = mujoco.MjData(self.mjModel)
        self.viewer = mujoco.viewer.launch_passive(
            self.mjModel, self.mjData, show_left_ui=False, show_right_ui=False
            )

    def step(self, action):
        self.mjData.ctrl = np.array([0. ,action])
        mujoco.mj_step(self.mjModel, self.mjData)

        obs = self.mjData.qpos, self.mjData.qvel

        return obs
    
    def render(self):
        self.viewer.sync()


if __name__ == "__main__":
    sim = MujocoSimulation()
    while True:
        sim.step(0.0)
        sim.render()