# Gazebo UR3e simulation

This is for simulating UR3e robot with Gazebo in a Jazzy ROS2 container. It does not use docker file and instead launches a docker terminal off of the bare image and so you can add further customization.

 URDFs and Meshes are from [here](https://github.com/Daniella1/urdf_files_dataset/blob/main/urdf_files/ros-industrial/xacro_generated/universal_robots/ur_description/urdf/ur3e.urdf).


You need to install docker (if you don't have it already). Once that is done, you can do the following steps (in linux bash terminal).

## Setup the Container

``` bash
# Move to this  directory
cd gazebo

# Pull ROS2 Jazzy image
sudo docker pull osrf/ros:jazzy-desktop-full

# Allow X11 forwarding
xhost +local:
```

## Run container

```bash
# Run image connected to current directory (which will be in /workspace directory in the container) and allow host to display gui using X11 Display Forwarding
sudo docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net host osrf/ros:jazzy-desktop-full bash
```


## Run Simulation

Container terminal should be started by this point.

```bash
cd workspace
gz sim world.sdf
```

Note: Jazzy has gazebo installed with the command `gz` instead of `gazebo`.



## Useful Commands

```bash
# Convert urdf to sdf
gz sdf -p robot.urdf > robot.sdf
```