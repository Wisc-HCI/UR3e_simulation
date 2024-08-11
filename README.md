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
