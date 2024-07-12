# the name of the target operating system
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86)

# set(CMAKE_FIND_ROOT_PATH /usr/lib32 /usr/local/lib32 /usr/local/boost_32)

# which compilers to use for C and C++
set(CMAKE_C_COMPILER arm-none-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-none-linux-gnueabihf-g++)

# Specify the Boost root, include, and library directories
set(BOOST_ROOT /opt/boost-arm)
set(BOOST_INCLUDEDIR /opt/boost-arm/include)
set(BOOST_LIBRARYDIR /opt/boost-arm/lib)

# where is the target environment located
# set(CMAKE_FIND_ROOT_PATH  /usr/i586-mingw32msvc /home/alex/mingw-install)

# adjust the default behavior of the FIND_XXX() commands:
# search programs in the host environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# search headers and libraries in the target environment
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
