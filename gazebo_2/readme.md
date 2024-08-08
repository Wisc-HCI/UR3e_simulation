## Starting with ubuntu image and adding ROS

```bash
# Install container image
docker pull ubuntu

# Start container connected to current directory (in container's  /workspace directory)
sudo docker run -it -v $(pwd):/workspace ubuntu bash
```

Below taken from [ros.org](
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

```bash
# Set locale settings
apt updat && apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable required repos
apt install software-properties-common
add-apt-repository universe

# Install ROS2
apt update &&  apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add to sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add dev tools
apt update &&  apt install ros-dev-tools

# Final update + install
apt update && apt upgrade && apt install ros-jazzy-desktop
```


## Use Image with ROS2 jazzy already setup
```bash
# Pull image
sudo docker pull osrf/ros:jazzy-desktop-full

xhost +local:

# Run it connected to current directory (in /workspace directory in container) and allow host to display gui using X11 Display Forwarding
sudo docker run -it -v $(pwd):/workspace osrf/ros:jazzy-desktop-full bash

sudo docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net host osrf/ros:jazzy-desktop-full bash
```

Note: Jazzy has gazebo installed with the command `gz` instead of `gazebo`.

<!-- Some installs like jazzy don't come with gazebo so you'll need to install it. Directions come from [classic.gazebosim.org](https://classic.gazebosim.org/tutorials?tut=install_ubuntu).
```bash
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

apt-get install wget

wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

apt-get update

apt-get install gazebo11
apt-get install libgazebo11-dev

``` -->



# Setting up gazebo in jazzy based off of [theconstruct.ai](https://www.theconstruct.ai/gazebo-5-minutes-004-create-gazebo-model-using-sdf/)

```bash
cd workspace
mkdir ~/simulation_ws/src -p

source /opt/ros/jazzy/setup.bash 
source /usr/share/gz/setup.sh
```

```bash
cd workspace
gz sim test.sdf
```
