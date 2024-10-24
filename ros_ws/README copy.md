# Setup

## Setup
If you want to run this repo on a docker container (recommended), please follow the instructions in [DOCKER_README.md](DOCKER_README.md). If you do not want to run this on docker, please first follow the instruction in  [NON_DOCKER_README.md](/NON_DOCKER_README.md).

Now build and source ros:
```bash
source /opt/ros/iron/setup.bash
colcon build --symlink-install
. install/setup.bash
```
## Running
To run the RVIZ visualization, do the following
```bash
ros2 launch relaxed_ik_ros2 single_converter.launch.py
```

## Notes
* Anytime you change anything in this repo, you will need to rebuild and source the code:
    ```bash
    colcon build
    source install/setup.bash
   
    ```
## Sources
* Visualizing URDF in RVIZ:
    * https://aleksandarhaber.com/how-to-create-urdf-and-launch-files-in-ros2-and-display-them-in-rviz/
    * https://www.learnros2.com/ros/ros2-building-blocks/configurations/rviz-configuration
* Relaxed IK: https://github.com/uwgraphics/relaxed_ik_ros2

## Mya notes please ignore
```bash
cd src/relaxed_ik_ros2/relaxed_ik_core
cargo build
cd ../../..

cd ../..
. install/setup.bash  # source install/setup.bash
# ros2 launch relaxed_ik_ros2 demo.launch.py


chmod +x /workspace/src/relaxed_ik_ros2/scripts/relaxed_ik_rust.py 
chmod +x /workspace/src/relaxed_ik_ros2/scripts/relaxed_ik_rust.py 



ros2 launch urdf_visualization demo.launch.py


```


[ERROR] [launch]: Caught exception in launch (see debug for traceback): executable 'relaxed_ik_rust.py' not found on the libexec directory '/workspace/install/relaxed_ik_ros2/lib/relaxed_ik_ros2' 