# Use a lightweight Debian-based image as the base
FROM debian:buster

# Create a directory inside the container for your source code
WORKDIR /code

# Set up a mount point for the source code (this will be linked to your host directory)
VOLUME /code

# Enable multi-arch support and install essential tools
RUN dpkg --add-architecture armhf
RUN apt-get update
RUN apt-get install -y \
    build-essential \
    gcc-arm-linux-gnueabihf \
    g++-arm-linux-gnueabihf \
    libasan5-armhf-cross \
    pkg-config \
    wget

RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.9/julia-1.9.3-linux-x86_64.tar.gz -O ~/julia.tar.gz
RUN tar -C /opt/ -zxvf ~/julia.tar.gz
RUN ln -s /opt/julia-1.9.3/bin/julia /usr/local/bin/julia
ENV PATH="/opt/julia-1.9.3/bin:$PATH" 

RUN julia --project=/code/libs/chan/compiler/JuliaCompiler -e 'using Pkg; Pkg.instantiate()'

RUN wget https://github.com/Kitware/CMake/releases/download/v3.30.2/cmake-3.30.2-linux-x86_64.tar.gz -O ~/cmake.tar.gz
RUN tar -C ~/ -zxvf ~/cmake.tar.gz
RUN mv ~/cmake-3.30.2-linux-x86_64 /opt/cmake
ENV PATH="/opt/cmake/bin:$PATH" 

# Install the specific Boost libraries for ARM architecture
RUN apt-get install -y \
    libboost-thread-dev:armhf \
    libboost-timer-dev:armhf \
    libboost-chrono-dev:armhf \
    libboost-serialization-dev:armhf \
    libboost-filesystem-dev:armhf \
    libboost-system-dev:armhf \
    libboost-program-options-dev:armhf \
    libeigen3-dev:armhf

# Set the default command to open a bash shell
CMD ["/bin/bash"]
