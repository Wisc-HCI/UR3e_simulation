# Docker Setup

These are the instructions for We have only tested these instructions on Ubuntu and Windows.

### 1. Install and Start Docker
Make sure you have installed docker. You can install docker [here for windows](https://docs.docker.com/desktop/install/windows-install/) or [here for linux](https://docs.docker.com/desktop/install/linux/).

Now make sure docker is started.

### 2. Setup display forwarding

**If you are on windows...**
Install https://sourceforge.net/projects/vcxsrv/. Start XLaunch (from the VcXsrv program group), set display settings to multiple windows, and ensure "Disable access control" is checked.

**If you are linux...**
Set up display forwarding by running:
```bash
xhost +local:
```
### 3. Build and Start The Container
Now  build the container image and start the container. Make sure you are in this directories root directory. These commands use the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. These commands also allow the containers display to be forwarded to your host machine so that you can see it.

```bash
cd ros_ws
```

**If you are on linux...**
```bash
sudo docker build -t ros-container .
sudo docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host ros-container
```

**If you are on Windows...**
```bash
docker build -t ros-container .
docker run -it -e DISPLAY=host.docker.internal:0.0 -v ${PWD}:/workspace --net=host ros-container
```

You and your computer are all set up, you rock! Now you are ready to continue onto README.md.


### 3. Starting Container 
Now build and source ros:
```bash
source /opt/ros/iron/setup.bash
colcon build --symlink-install
. install/setup.bash
```
### 4.  Running
To run the RVIZ visualization, do the following
```bash

# Revision
ros2 launch relaxed_ik_ros2 ik.launch.py

ros2 launch relaxed_ik_ros2 script.launch.py # do in new terminal

```

## Notes
* Anytime you change anything in this repo, you will need to rebuild and source the code:
    ```bash
    colcon build
    source install/setup.bash
   
    ```

---

### Docker Tips
* To open another docker terminal for a running container, run the following on your home-machine:
    ```bash
    # Show your running CONTAINER_ID
    docker ps 

    # Open another terminal using that CONTAINER_ID
    docker exec -it  <YOUR_CONTAINER_ID> bash

    # Source ROS properly
    source /opt/ros/iron/setup.sh
    source install/setup.bash
    ```
