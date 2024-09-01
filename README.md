- To start relaxed_ik_ros2: refer to README.md in relaxed_ik_ros2. For detailed directions, see below.
- Note. When adding new nodes within the ``scripts`` folder, make sure to add filename under ``relaxed_ik_ros2 > CMakeLists.txt > install``, or else when launch, the node file mentioned in the launch file will not be found and hence throw an error.


## Running rviz and joint_states.py (or velocities.py) for simulation
First see the next 2 sections to set up relaxed_ik_ros2 and ros2 if needed. Then do the following.
You need to open 2 terminals. Whenever the ros2 command does not work, that means that ros2 is not sourced so do ``source /opt/ros/${ROS_DISTRO}/setup.bash``.

### When input is raw angular velocity - ``velocities.py`` and ``demo.launch.py``
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

### When input is joint state (6 values for 6-dof robot) - ``joint_states.py`` and ``rviz_joint_states.py``
Run similar commands but just replace the filenames to ``joint_states.py`` and ``rviz_joint_states.py`` for the script (``ros run``) and the launch file (``ros launch``), respectively.

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
