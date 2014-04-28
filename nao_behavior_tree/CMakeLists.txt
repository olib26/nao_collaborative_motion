cmake_minimum_required(VERSION 2.4.6)
project (NaoBehaviorTree)
message(Compiling: ${PROJECT_NAME})

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

# Turn off shared libs and turn on static libs, just for this package
set(ROS_BUILD_STATIC_LIBS true)
set(ROS_BUILD_SHARED_LIBS false)


rosbuild_init()


# Settings
add_definitions(-Wall -g -O0 -Wno-deprecated -static -Bstatic -std=gnu++0x)

# Includes
set(COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/src)
include_directories(${COMMON_INCLUDES})

# Project source
file(GLOB_RECURSE SRC ${PROJECT_SOURCE_DIR}/src/behavior_tree/*.cpp)

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

#########################################################
# FIND OPENCV
#########################################################
find_package(OpenCV 2.4.8 REQUIRED)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


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


rosbuild_add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_boost_directories()


# Actions
rosbuild_add_executable(Greet src/actions/Greet.cpp src/behavior_tree/rosaction.cpp)
target_link_libraries(Greet alproxies alvalue)
rosbuild_link_boost(Greet thread)

rosbuild_add_executable(Stop src/actions/Stop.cpp src/behavior_tree/rosaction.cpp)
target_link_libraries(Stop alproxies alvalue)
rosbuild_link_boost(Stop thread)

rosbuild_add_executable(Search src/actions/Search.cpp src/behavior_tree/rosaction.cpp src/filters/particleFilter.cpp)
target_link_libraries(Search alproxies alvalue ${OpenCV_LIBS})
rosbuild_link_boost(Search thread)

rosbuild_add_executable(GoClose src/actions/GoClose.cpp src/behavior_tree/rosaction.cpp src/filters/particleFilter.cpp)
target_link_libraries(GoClose alproxies alvalue ${OpenCV_LIBS})
rosbuild_link_boost(GoClose thread)

rosbuild_add_executable(TrackingAmongObstacles src/actions/TrackingAmongObstacles.cpp src/behavior_tree/rosaction.cpp src/filters/particleFilter.cpp)
target_link_libraries(TrackingAmongObstacles alproxies alvalue ${OpenCV_LIBS})
rosbuild_link_boost(TrackingAmongObstacles thread)

rosbuild_add_executable(HeadTracker src/actions/HeadTracker.cpp src/behavior_tree/rosaction.cpp)
target_link_libraries(HeadTracker alproxies alvalue)
rosbuild_link_boost(HeadTracker thread)

rosbuild_add_executable(GoToPoint src/actions/GoToPoint.cpp src/behavior_tree/rosaction.cpp)
target_link_libraries(GoToPoint alproxies alvalue)
rosbuild_link_boost(GoToPoint thread)

rosbuild_add_executable(Walk src/actions/Walk.cpp src/behavior_tree/rosaction.cpp)
target_link_libraries(Walk alproxies alvalue)
rosbuild_link_boost(Walk thread)

rosbuild_add_library(bt_server ${PROJECT_SOURCE_DIR}/src/behavior_tree/rosaction.cpp)


# Map
rosbuild_add_executable(Localization src/map/localization.cpp src/behavior_tree/rosaction.cpp src/filters/particleFilter.cpp)
target_link_libraries(Localization ${OpenCV_LIBS} alproxies alvalue alvision)
rosbuild_link_boost(Localization thread)

rosbuild_add_executable(Obstacles src/map/obstacles.cpp src/behavior_tree/rosaction.cpp)
target_link_libraries(Obstacles ${OpenCV_LIBS} alproxies alvalue alvision)
rosbuild_link_boost(Obstacles thread)


# Sensors
rosbuild_add_executable(nao_sonar src/nao/nao_sonar.cpp)
target_link_libraries(nao_sonar alproxies alvalue)


# Leds
rosbuild_add_executable(nao_leds src/nao/nao_leds.cpp)
target_link_libraries(nao_leds alproxies alvalue)


# Tools
rosbuild_add_executable(color_filter_calibration src/tools/color_filter_calibration.cpp)
target_link_libraries(color_filter_calibration ${OpenCV_LIBS})
