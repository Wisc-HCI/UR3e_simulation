
To run Lively Studio, I've created a container image with all the dependencies. I have only tested this on Linux Ubuntu.

<!-- sudo docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net host ubuntu:20.04 bash -->
## Setup + Running

Run the following commands to build the image and start the container:


```bash
cd lively_testing  # Get to this directory (may need another command)
sudo docker build -t lively-image .
sudo docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host lively-image

# Now your container should be running
cd LivelyStudio

yarn install
yarn start


```



## Resources
https://github.com/Wisc-HCI/LivelyStudio?tab=readme-ov-file
https://wisc-hci.github.io/lively/docs/Tutorials/solving