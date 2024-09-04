# Use an official Ubuntu 20.04 LTS as a parent image
FROM ubuntu:20.04

# Set noninteractive to avoid prompts during the build
ARG DEBIAN_FRONTEND=noninteractive

# Update apt package list and install general packages
RUN apt-get update && \
    apt-get install -y \
    git-all \
    curl\
    wget \
    python3-pip \
    iputils-ping

# Install lively-specific apt packages
RUN  apt-get install -y libwebkit2gtk-4.0-dev \
    build-essential \
    file \
    libssl-dev \
    libgtk-3-dev \
    libayatana-appindicator3-dev \
    librsvg2-dev

# Install node related dependencies of lively
# Install Node.js from Nodesource to get the latest version
RUN curl -sL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs
RUN apt-get install npm -y 
RUN npm install -g yarn

# Install lively rust dependency
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"


WORKDIR workspace


# Set the default command to execute
# When creating a container, this will simulate `docker run -it`
CMD ["bash"]