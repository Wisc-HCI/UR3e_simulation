TODO: make rviz represent linear/angular velocities defined in relaxed_ik_ros2/scripts/velocities.py
- Problem: rviz runs with urdf, etc (see scripts/demo.launch.py) however the robot is not moving, although it should in alignment to whatever velocities.py is making it to do so. Specific directions on how to run is also given below.
- To start relaxed_ik_ros2: refer to README.md in relaxed_ik_ros2. For detailed directions, see below.
- Note. currently, the setup is done with UR5. You can change this to UR3e by relaxed_ik_ros2/relaxed_ik_core/configs/settings.yaml.


## Running rviz and velocities.py for simulation
First see the next 2 sections to set up relaxed_ik_ros2 and ros2 if needed. Then do the following.
You need to open 2 terminals. Whenever the ros2 command does not work, that means that ros2 is not sourced so do ``source /opt/ros/${ROS_DISTRO}/setup.bash``.

```bash
# in one terminal, fire up the launch python wrapper file
cd src
ros2 launch relaxed_ik_ros2 demo.launch.py # rviz will pop up and the robot will be shown. There should be a blue light on for the RobotModel (meaning no errors)

# in another terminal, run the Python node (velocities.py)
. install/setup.bash
source/opt/ros/${ROS_DISTRO}/setup.bash
ros2 run relaxed_ik_ros2 velocities.py
```
Whenever changes are made to velocities.py or to relevant files in the relaxed_ik_ros2 package, the package must be built and compiled everytime (`colcon build`  +  `. install/setup.bash`).

If you need a reference on how velocities.py (should) operate, then you can run 
```bash
ros2 run relaxed_ik_ros2 keyboard_input.py
```
and use keyboard keys to control the velocities/angles in rviz, in real-time.

## Starting relaxed_ik_ros2
For the bulk of the instructions, follow the README in the relaxed_ik_ros2 folder. 
The UR description files are already in there, so you just have to install it.
Installation: https://automaticaddison.com/how-to-create-a-urdf-file-of-the-ur3e-robotic-arm-ros-2/

```bash
# Intall python packages
pip3 install -r requirements.txt

# install the required packages
sudo apt-get update
sudo apt install python3-pykdl
sudo apt-get install ros-${ROS_DISTRO}-urdfdom-py
sudo apt-get install ros-${ROS_DISTRO}-ur-description

# verify installation via rviz (for example, UR3e)
source /opt/ros/${ROS_DISTRO}/setup.bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur3e


```

Whenever you are building the relaxed_ik_ros2 package, use ``colcon build``.
Specifically, make sure you are in your workstation and not in src and run 
```bash
colcon build --packages-select relaxed_ik_ros2 --symlink-install # will take about 3 ~ 8 seconds
. install/setup.bash
```

## ROS2 Iron Installation

```bash
# Follow general instructions from [the official ROS2 website](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)

## Setup locale
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

## Ensure that the Ubuntu Universal Repository is enabled
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

## Skip the ROS2 GPG key adding part in the instructions. Instead, refer to post [here](https://answers.ros.org/question/410123/ubuntu-2204-ros2-humble-installing-error-gpg-libc-bin/).
rm /etc/apt/sources.list.d/ros2.list # if exists
curl http://repo.ros2.org/repos.key | sudo apt-key add -
sudo apt-key list # find key name
sudo cp /etc/apt/trusted.gpg /usr/share/keyrings/ros-archive-keyring.gpg

## go back to general instructions
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-dev-tools

sudo apt update

sudo apt upgrade

sudo apt install ros-iron-desktop

source /opt/ros/iron/setup.bash
```
