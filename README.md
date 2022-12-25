## Overview
This repo contains the code for controlling both a real and a simulated flywheel pendulum robot via ROS2. It includes the following folders:

1. ```python_scripts```: most of the ROS2 nodes call some classes here
 
2. ```simulations```: scenes used for simulating the robot with CoppeliaSim and Gazebo Garden

3. ```ros2_ws```: collection of ROS2 nodes for controlling the robot

4. ```docker_files```: collection of docker images for intel/nvidia/wsl


## Dependencies
1. [ROS2](https://docs.ros.org/en/humble/Installation.html) Humble

2. [CoppeliaSim](https://www.coppeliarobotics.com/downloads) and [Gazebo Garden](https://gazebosim.org/home)(TO DO) for simulations 


## Build on Linux
1. clone the repo
```sh
git clone --recurse-submodules https://github.com/giulioturrisi/Flywheel-Pendulum-Robot.git
```

2. build the docker file inside Flywheel-Pendulum-Robot/docker_file/integrated_gpu or /nvidia
```sh
docker build -t ros2_flywheel .
```

4. add alias to start the docker
```sh
cd 
gedit .bashrc
alias flywheel_humble='xhost + && docker run -it --rm -v /path/to/your_folder/Flywheel-Pendulum-Robot:/home/ -v /tmp/.X11-unix:/tmp/.X11-unix:rw --device=/dev/input/ -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY  -e QT_X11_NO_MITSHM=1 --gpus all --name flywheel_image ros2_flywheel'  (if used /nvidia)
alias flywheel_humble="xhost + && docker run -it --rm -v /path/to/your_folder/Flywheel-Pendulum-Robot:/home/ -v /tmp/.X11-unix:/tmp/.X11-unix --device=/dev/dri --device=/dev/input/ -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY --name flywheel_image  ros2_flywheel" (if used /integrated_gpu)
alias flywheel_humble='xhost + && docker run -it --rm -v /path/to/your_folder/Flywheel-Pendulum-Robot:/home/ -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e PULSE_SERVER=$PULSE_SERVER -e LD_LIBRARY_PATH=/usr/lib/wsl/lib --name flywheel_image ros2_flywheel' (if Windows Linux Subsystem)

alias flywheel='docker exec -it flywheel_image bash' (to attach a new terminal to the running docker)
```

5. start docker and build
```sh
flywheel_humble
cd ros2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
ulimit -s unlimited
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

6. to attach additional terminals to the running docker use
```sh
flywheel
```

## List of available controllers
1. Feedback Linearization
2. Linear Quadratic Regulator
3. Adaptive Linear Quadratic Regulator
4. Nonlinear MPC via Acados
5. Iterative Linear Quadratic Regulator


## Real Robot
The CAD files are from the wonderful project based on [SimpleFoc](https://github.com/simplefoc/Arduino-FOC-reaction-wheel-inverted-pendulum)


## Some Gifs
<p align="center">
    <img src="gifs/flywheel_robot_sim2.gif" height="200px">
    <img src="gifs/flywheel_robot_sim.gif" height="205px">
</p>

