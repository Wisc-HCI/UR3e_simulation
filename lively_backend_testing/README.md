
To run this code, I've created a container image with all the dependencies. I have only tested this on Linux Ubuntu.

## Setup + Running

Run the following commands to build the image and start the container:


```bash
cd lively_backend_testing  # Get to this directory (may need another command)
sudo docker build -t backend-image .
sudo docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host backend-image

# Now your container should be running
cd LivelyStudio

yarn install
yarn start


```



## Resources
https://github.com/Wisc-HCI/LivelyStudio?tab=readme-ov-file
https://wisc-hci.github.io/lively/docs/Tutorials/solving