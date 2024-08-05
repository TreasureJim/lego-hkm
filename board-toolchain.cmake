# toolchain.cmake for ARMv7 cross-compilation
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_PROCESSOR arm)

# Specify the cross compiler
SET(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

# Where to look for the target environment (headers/libs)
# SET(CMAKE_FIND_ROOT_PATH /usr/arm-linux-gnueabihf)

# Specify the Boost root, include, and library directories
set(BOOST_LIBRARYDIR /usr/lib/x86_64-linux-gnu)

# Adjust the default behavior of the FIND_XXX() commands:
# Search for programs in the host environment
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# Search for libraries and headers in the target environment
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Set the flags for cross-compilation
SET(CMAKE_C_FLAGS "-mfpu=vfp -mfloat-abi=hard -march=armv7-a" CACHE STRING "" FORCE)
SET(CMAKE_CXX_FLAGS "-mfpu=vfp -mfloat-abi=hard -march=armv7-a" CACHE STRING "" FORCE)
