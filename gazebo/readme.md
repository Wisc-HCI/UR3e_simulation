


```bash
sudo docker build -t gazebo:latest .
```

```bash
xhost +local:root    # This allows the local machine to display GUI from Docker
sudo docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix gazebo:latest
```


<!-- ```bash
sudo docker build -t ros-gazebo-urdf .
```

```bash
sudo docker run -it --rm ros-gazebo-urdf:latest
``` -->
