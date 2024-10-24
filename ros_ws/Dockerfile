# Use an official Ubuntu 20.04 LTS as a parent image
FROM osrf/ros:iron-desktop-full

# Set noninteractive to avoid prompts during the build
ARG DEBIAN_FRONTEND=noninteractive

# INSTALL PACKAGES
RUN apt-get update && \
    apt-get install -y \
    curl\
    python3-pip\
    wget\
    ros-iron-joint-state-publisher \
    ros-iron-joint-state-publisher-gui \
    ros-iron-xacro\
    python3-pykdl\
    vim


# Install python packages

RUN pip install urdf-parser-py\
    readchar\
    pynput\
    pandas

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

WORKDIR /workspace/

# Set the default command to execute
# When creating a container, this will simulate `docker run -it`
CMD ["bash"]

