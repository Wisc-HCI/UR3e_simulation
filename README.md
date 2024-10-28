- To start relaxed_ik_ros2: refer to README.md in relaxed_ik_ros2. For detailed directions, see below.
- Note. When adding new nodes within the ``scripts`` folder, make sure to add filename under ``relaxed_ik_ros2 > CMakeLists.txt > install``, or else when launch, the node file mentioned in the launch file will not be found and hence throw an error.

## Running ``cartesian_to_joint.py`` and ``cartesian_to_joint_multi.py``
First start ``RelaxedIK`` by running the ``single_converter.launch.py`` launch file. Then at a new terminal, run either the ``cartesian_to_joint.py`` ROS node or the multi-version of that (i.e., running multiple coordinate csv files sequentially), ``cartesian_to_joint_multi.py``. Make sure the modify the launch file accordingly to your choice of ``cartesian_xx`` ROS node file.


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
## Troubleshooting build error
``colcon build`` sometimes yield ``no rosidl_typesupport_c found`` error, even though the ``CMakeLists.txt`` includes finding the appropriate ``rosidl`` packages. You can clear out the previous cmake caches using the option ``--cmake-clean-cache`` however, this leads to a ``libexec`` error, where the ROS nodes in the ``install`` folder are not executable, hence build may be successful however not work when running the ROS node. You should instead try ``colcon bulid --cmake-clean-cache --symlink-install --packages-select relaxed_ik_ros2`` so that the ``symlink`` correctly forms the files in the ``install`` folder as executable. An ``ament-package-folder already exists`` error that is yielded from this command can then be easily fixed by ``rm -rf``-ing the existing ``install`` folders, and running ``colcon build --symlink-install --packages-select relaxed_ik_ros2`` subsequently.

 
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
