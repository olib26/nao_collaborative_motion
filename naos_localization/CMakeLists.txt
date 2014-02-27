cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()


# Settings
add_definitions(-Wall -g -O0 -Wno-deprecated -static -Bstatic -std=gnu++0x)

# Includes
set(COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/src)
include_directories(${COMMON_INCLUDES})

#########################################################
# FIND OPENCV
#########################################################
find_package(OpenCV 2.4.8 REQUIRED)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


# Aldebaran libraries
include_directories(
$ENV{AL_DIR_SDK}/include
$ENV{AL_DIR_SDK}/include/alcommon
$ENV{AL_DIR_SDK}/include/alvision
$ENV{AL_DIR_SDK}/include/alcore
$ENV{AL_DIR_SDK}/include/alproxies
$ENV{AL_DIR_SDK}/include/alvalue
$ENV{AL_DIR_SDK}/include/altools
$ENV{AL_DIR_SDK}/include/alfactory
$ENV{AL_DIR_SDK}/include/libthread
$ENV{AL_DIR_SDK}/include/alvision
)

link_directories(
$ENV{AL_DIR_SDK}/lib
)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)


rosbuild_add_executable(main src/main.cpp)
target_link_libraries(main ${OpenCV_LIBS} alproxies alvalue alvision)


#target_link_libraries(example ${PROJECT_NAME})
