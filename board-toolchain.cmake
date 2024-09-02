# toolchain.cmake for ARMv7 cross-compilation
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_PROCESSOR arm)

# Specify the cross compiler
SET(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

# Where to look for the target environment (headers/libs)
# SET(CMAKE_FIND_ROOT_PATH /usr/arm-linux-gnueabihf)

# Specify the Boost root, include, and library directories
set(BOOST_ROOT /usr)
set(BOOST_LIBRARYDIR /usr/lib/arm-linux-gnueabihf)
# set(BOOST_DEBUG ON)

# Set the flags for cross-compilation
SET(CMAKE_C_FLAGS "-mfpu=vfp -mfloat-abi=hard -march=armv7-a" CACHE STRING "" FORCE)
SET(CMAKE_CXX_FLAGS "-mfpu=vfp -mfloat-abi=hard -march=armv7-a" CACHE STRING "" FORCE)
