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
CMAKE_SOURCE_DIR = /home/olivier/ros_workspace/first_approaches

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/olivier/ros_workspace/first_approaches

# Include any dependencies generated for this target.
include CMakeFiles/nao_detection.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/nao_detection.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/nao_detection.dir/flags.make

CMakeFiles/nao_detection.dir/src/nao_detection.o: CMakeFiles/nao_detection.dir/flags.make
CMakeFiles/nao_detection.dir/src/nao_detection.o: src/nao_detection.cpp
CMakeFiles/nao_detection.dir/src/nao_detection.o: manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/share/actionlib_msgs/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/share/trajectory_msgs/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/share/nav_msgs/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /home/olivier/ros_workspace/nao/stacks/nao_robot/nao_msgs/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/nao_detection.dir/src/nao_detection.o: /home/olivier/ros_workspace/nao/stacks/nao_robot/nao_msgs/msg_gen/generated
CMakeFiles/nao_detection.dir/src/nao_detection.o: /home/olivier/ros_workspace/nao/stacks/nao_robot/nao_msgs/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/olivier/ros_workspace/first_approaches/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/nao_detection.dir/src/nao_detection.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/nao_detection.dir/src/nao_detection.o -c /home/olivier/ros_workspace/first_approaches/src/nao_detection.cpp

CMakeFiles/nao_detection.dir/src/nao_detection.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nao_detection.dir/src/nao_detection.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/olivier/ros_workspace/first_approaches/src/nao_detection.cpp > CMakeFiles/nao_detection.dir/src/nao_detection.i

CMakeFiles/nao_detection.dir/src/nao_detection.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nao_detection.dir/src/nao_detection.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/olivier/ros_workspace/first_approaches/src/nao_detection.cpp -o CMakeFiles/nao_detection.dir/src/nao_detection.s

CMakeFiles/nao_detection.dir/src/nao_detection.o.requires:
.PHONY : CMakeFiles/nao_detection.dir/src/nao_detection.o.requires

CMakeFiles/nao_detection.dir/src/nao_detection.o.provides: CMakeFiles/nao_detection.dir/src/nao_detection.o.requires
	$(MAKE) -f CMakeFiles/nao_detection.dir/build.make CMakeFiles/nao_detection.dir/src/nao_detection.o.provides.build
.PHONY : CMakeFiles/nao_detection.dir/src/nao_detection.o.provides

CMakeFiles/nao_detection.dir/src/nao_detection.o.provides.build: CMakeFiles/nao_detection.dir/src/nao_detection.o

# Object files for target nao_detection
nao_detection_OBJECTS = \
"CMakeFiles/nao_detection.dir/src/nao_detection.o"

# External object files for target nao_detection
nao_detection_EXTERNAL_OBJECTS =

bin/nao_detection: CMakeFiles/nao_detection.dir/src/nao_detection.o
bin/nao_detection: CMakeFiles/nao_detection.dir/build.make
bin/nao_detection: CMakeFiles/nao_detection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/nao_detection"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/nao_detection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/nao_detection.dir/build: bin/nao_detection
.PHONY : CMakeFiles/nao_detection.dir/build

CMakeFiles/nao_detection.dir/requires: CMakeFiles/nao_detection.dir/src/nao_detection.o.requires
.PHONY : CMakeFiles/nao_detection.dir/requires

CMakeFiles/nao_detection.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/nao_detection.dir/cmake_clean.cmake
.PHONY : CMakeFiles/nao_detection.dir/clean

CMakeFiles/nao_detection.dir/depend:
	cd /home/olivier/ros_workspace/first_approaches && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/olivier/ros_workspace/first_approaches /home/olivier/ros_workspace/first_approaches /home/olivier/ros_workspace/first_approaches /home/olivier/ros_workspace/first_approaches /home/olivier/ros_workspace/first_approaches/CMakeFiles/nao_detection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/nao_detection.dir/depend

