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

# Source: https://stackoverflow.com/questions/36399848/install-node-in-dockerfile
# MAY NEED TO UPDATE THIS VERSION LATER ON
ENV NODE_VERSION=20.17.0
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
ENV NVM_DIR=/root/.nvm
RUN . "$NVM_DIR/nvm.sh" && nvm install ${NODE_VERSION}
RUN . "$NVM_DIR/nvm.sh" && nvm use v${NODE_VERSION}
RUN . "$NVM_DIR/nvm.sh" && nvm alias default v${NODE_VERSION}
ENV PATH="/root/.nvm/versions/node/v${NODE_VERSION}/bin/:${PATH}"
RUN npm install -g yarn


WORKDIR workspace


# Set the default command to execute
# When creating a container, this will simulate `docker run -it`
CMD ["bash"]