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
            self.mjModel, self.mjData, show_left_ui=False, show_right_ui=True
            )
        
        # Reset the simulation
        mujoco.mj_resetDataKeyframe(self.mjModel, self.mjData, 0)

    def step(self, action):
        self.mjData.ctrl = np.array([0., action])
        mujoco.mj_step(self.mjModel, self.mjData)

 
        obs = np.array([self.mjData.qpos[0], self.mjData.qvel[0], self.mjData.qvel[1]])
        

        # Wrap the angle between -3.14 and 3.14
        obs[0] = np.mod(obs[0] + np.pi, 2 * np.pi) - np.pi

        # Shift the 0 angle to -3.14 and 3.14
        if obs[0] > 0:
            obs[0] = np.pi - obs[0]
        else:
            obs[0] = -np.pi - obs[0]
        
        obs[0] = -obs[0]
        print("obs: ", obs)

        return obs
    
    def render(self):
        self.viewer.sync()


if __name__ == "__main__":
    sim = MujocoSimulation()
    while True:
        sim.step(0.0)
        sim.render()