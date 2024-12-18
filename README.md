## Overview
This repo contains the code for controlling both a real and a simulated (in [Mujoco](https://github.com/google-deepmind/mujoco) or [CoppeliaSim](https://www.coppeliarobotics.com/downloads)) flywheel pendulum robot via ROS2. 

## List of available controllers
1. Feedback Linearization
2. Linear Quadratic Regulator
3. Adaptive Linear Quadratic Regulator
4. Nonlinear MPC via Acados
4. Nonlinear MPC via Casadi
6. Iterative Linear Quadratic Regulator

## Repository structure
It includes the following folders:

1. ```python_scripts```: most of the ROS2 nodes call some classes here
 
2. ```coppeliasim_simulation```: scenes used for simulating the robot with CoppeliaSim

3. ```ros2_ws```: collection of ROS2 nodes for controlling the robot



## Dependencies
1. [ROS2](https://docs.ros.org/en/humble/Installation.html) Humble

2. [CoppeliaSim](https://www.coppeliarobotics.com/downloads) or [Mujoco](https://github.com/google-deepmind/mujoco)


## Build on Linux
1. clone the repo recursively

```sh
git clone --recurse-submodules https://github.com/giulioturrisi/Flywheel-Pendulum-Robot.git
```


2. install [miniforge](https://github.com/conda-forge/miniforge/releases) (x86_64) 


3. create an environment using the file in the folder [installation/conda](https://github.com/giulioturrisi/Flywheel-Pendulum-Robot/tree/master/installation/conda):

```sh
    conda env create -f mamba_environment.yml
``` 

4. follow the instruction [here](https://robostack.github.io/GettingStarted.html) to install ros-humble


5. download [CoppeliaSim](https://www.coppeliarobotics.com/) (if using Mujoco, this is not needed)

6. add in your .bashrc

```sh
alias flywheel_env="conda activate flywheel_env && source your_path_to/Flywheel-Pendulum-Robot/ros2_ws/install/setup.bash"
export COPPELIASIM_ROOT_DIR=your_path_to/CoppeliaSim (if using Mujoco, this is not needed)
```

7. start your environment and go in ros2_ws
```sh
twip_env
cd your_path_to/Flywheel-Pendulum-Robot/ros2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
ulimit -s unlimited
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

8. if you need acados, go inside the [acados](https://github.com/giulioturrisi/Flywheel-Pendulum-Robot/tree/master/python_scripts/controllers/acados)/acados folder and press
  
```sh
mkdir build
cd build
cmake -DACADOS_WITH_QPOASES=ON  -DACADOS_WITH_OSQP=ON ..
make install -j4
pip install -e ./../interfaces/acados_template
```
then in your .bashrc, add
```sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/your_path_to/Flywheel-Pendulum-Robot/python_scripts/controllers/acados/lib"
export ACADOS_SOURCE_DIR="/your_path_to/Flywheel-Pendulum-Robot/python_scripts/controllers/acados"
```

## How to run the simulation
1. Open Coppeliasim and run the scene `scene.ttt` in the folder coppeliasim_simulation 
```sh
./coppeliaSim.sh -f your_path_to/Flywheel-Pendulum-Robot/coppeliasim_simulation/scene.ttt 
```

or if you use Mujoco

```sh
ros2 run simulations run_mujoco_simulation.py 
```

2. on a new terminal 
```sh
ros2 run controllers <control_node>                     
```
where in <control_node> you can choose the type of controller you want. 




## Real Robot
The CAD files are from the wonderful project based on [SimpleFoc](https://github.com/simplefoc/Arduino-FOC-reaction-wheel-inverted-pendulum)


