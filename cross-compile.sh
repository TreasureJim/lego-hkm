#!/bin/bash

# Define your source directory (the location of your CMakeLists.txt)
SOURCE_DIR=$(pwd)

# Run the Docker container and compile the project
docker run --rm -v ${SOURCE_DIR}:/code armv7-cross-compile-env-boost-eigen \
    bash -c "cd /code && mkdir -p board && cd board && cmake .. -D CMAKE_TOOLCHAIN_FILE=/code/board-toolchain.cmake -D BUILD_PROD=ON && make -j4"
