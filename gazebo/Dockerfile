# Use Ubuntu base image
FROM ubuntu:20.04

# Avoid user interaction with tzdata
ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo
RUN apt-get update && apt-get install -y \
    wget \
    lsb-release \
    gnupg \
    && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y gazebo11 \
    && rm -rf /var/lib/apt/lists/*

# Setup environment
ENV DISPLAY=:0

# Create a directory for your robot files
WORKDIR /

# Copy your URDF and meshes into the container
COPY ./urdf ./urdf/

# Command to run when starting the container
CMD ["gazebo", "--verbose", "urdf/simulation_test.sdf"]




# # Use an official ROS base image
# FROM osrf/ros:noetic-desktop-full

# # Install dependencies
# RUN apt-get update && apt-get install -y \
#     ros-noetic-urdf \
#     ros-noetic-gazebo-ros \
#     ros-noetic-gazebo-ros-control \
#     ros-noetic-gazebo-ros-pkgs
#     # ros-noetic-xacro \

# # Setup environment
# RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# # Set the working directory
# WORKDIR /root/catkin_ws
# RUN mkdir src

# # Initialize ROS workspace
# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# # Copy your URDF files into the container (in UR3e_SIMULATION package)
# # Copy all files into container
# COPY . /root/catkin_ws/src/UR3e_SIMULATION

# # Launch Gazebo with the URDF file on container startup
# CMD ["roslaunch", "UR3e_SIMULATION", "display.launch"]
