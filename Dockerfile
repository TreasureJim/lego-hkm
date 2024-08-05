# Use a lightweight Debian-based image as the base
FROM debian:buster

# Enable multi-arch support and install essential tools
RUN dpkg --add-architecture armhf && \
    apt-get update && \
    apt-get install -y \
    build-essential \
    gcc-arm-linux-gnueabihf \
    g++-arm-linux-gnueabihf \
    cmake \
    pkg-config

# Install the specific Boost libraries for ARM architecture
RUN apt-get update && \
    apt-get install -y \
    libboost-thread-dev:armhf \
    libboost-timer-dev:armhf \
    libboost-chrono-dev:armhf \
    libboost-serialization-dev:armhf \
    libboost-filesystem-dev:armhf \
    libboost-system-dev:armhf \
    libboost-program-options-dev:armhf \
    libeigen3-dev:armhf

# Create a directory inside the container for your source code
WORKDIR /code

# Set up a mount point for the source code (this will be linked to your host directory)
VOLUME /code

# Set the default command to open a bash shell
CMD ["/bin/bash"]
