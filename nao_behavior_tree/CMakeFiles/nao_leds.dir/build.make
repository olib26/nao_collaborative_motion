# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/olivier/ros_workspace/nao_behavior_tree

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/olivier/ros_workspace/nao_behavior_tree

# Include any dependencies generated for this target.
include CMakeFiles/nao_leds.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/nao_leds.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/nao_leds.dir/flags.make

CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: CMakeFiles/nao_leds.dir/flags.make
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: src/nao/nao_leds.cpp
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/actionlib_msgs/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/actionlib/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/nav_msgs/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /opt/ros/fuerte/share/trajectory_msgs/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /home/olivier/ros_workspace/nao/stacks/nao_robot/nao_msgs/manifest.xml
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /home/olivier/ros_workspace/nao/stacks/nao_robot/nao_msgs/msg_gen/generated
CMakeFiles/nao_leds.dir/src/nao/nao_leds.o: /home/olivier/ros_workspace/nao/stacks/nao_robot/nao_msgs/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/olivier/ros_workspace/nao_behavior_tree/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/nao_leds.dir/src/nao/nao_leds.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/nao_leds.dir/src/nao/nao_leds.o -c /home/olivier/ros_workspace/nao_behavior_tree/src/nao/nao_leds.cpp

CMakeFiles/nao_leds.dir/src/nao/nao_leds.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nao_leds.dir/src/nao/nao_leds.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/olivier/ros_workspace/nao_behavior_tree/src/nao/nao_leds.cpp > CMakeFiles/nao_leds.dir/src/nao/nao_leds.i

CMakeFiles/nao_leds.dir/src/nao/nao_leds.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nao_leds.dir/src/nao/nao_leds.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/olivier/ros_workspace/nao_behavior_tree/src/nao/nao_leds.cpp -o CMakeFiles/nao_leds.dir/src/nao/nao_leds.s

CMakeFiles/nao_leds.dir/src/nao/nao_leds.o.requires:
.PHONY : CMakeFiles/nao_leds.dir/src/nao/nao_leds.o.requires

CMakeFiles/nao_leds.dir/src/nao/nao_leds.o.provides: CMakeFiles/nao_leds.dir/src/nao/nao_leds.o.requires
	$(MAKE) -f CMakeFiles/nao_leds.dir/build.make CMakeFiles/nao_leds.dir/src/nao/nao_leds.o.provides.build
.PHONY : CMakeFiles/nao_leds.dir/src/nao/nao_leds.o.provides

CMakeFiles/nao_leds.dir/src/nao/nao_leds.o.provides.build: CMakeFiles/nao_leds.dir/src/nao/nao_leds.o

# Object files for target nao_leds
nao_leds_OBJECTS = \
"CMakeFiles/nao_leds.dir/src/nao/nao_leds.o"

# External object files for target nao_leds
nao_leds_EXTERNAL_OBJECTS =

bin/nao_leds: CMakeFiles/nao_leds.dir/src/nao/nao_leds.o
bin/nao_leds: /usr/local/lib/libopencv_calib3d.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_contrib.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_core.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_features2d.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_flann.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_gpu.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_highgui.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_imgproc.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_legacy.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_ml.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_nonfree.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_objdetect.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_photo.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_stitching.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_ts.a
bin/nao_leds: /usr/local/lib/libopencv_video.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_videostab.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_nonfree.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_ocl.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_gpu.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_legacy.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_calib3d.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_features2d.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_flann.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_ml.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_objdetect.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_highgui.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_photo.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_video.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_imgproc.so.2.4.8
bin/nao_leds: /usr/local/lib/libopencv_core.so.2.4.8
bin/nao_leds: CMakeFiles/nao_leds.dir/build.make
bin/nao_leds: CMakeFiles/nao_leds.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/nao_leds"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/nao_leds.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/nao_leds.dir/build: bin/nao_leds
.PHONY : CMakeFiles/nao_leds.dir/build

CMakeFiles/nao_leds.dir/requires: CMakeFiles/nao_leds.dir/src/nao/nao_leds.o.requires
.PHONY : CMakeFiles/nao_leds.dir/requires

CMakeFiles/nao_leds.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/nao_leds.dir/cmake_clean.cmake
.PHONY : CMakeFiles/nao_leds.dir/clean

CMakeFiles/nao_leds.dir/depend:
	cd /home/olivier/ros_workspace/nao_behavior_tree && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/olivier/ros_workspace/nao_behavior_tree /home/olivier/ros_workspace/nao_behavior_tree /home/olivier/ros_workspace/nao_behavior_tree /home/olivier/ros_workspace/nao_behavior_tree /home/olivier/ros_workspace/nao_behavior_tree/CMakeFiles/nao_leds.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/nao_leds.dir/depend

