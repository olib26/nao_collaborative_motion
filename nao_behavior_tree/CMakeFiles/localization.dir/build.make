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
include CMakeFiles/localization.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/localization.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/localization.dir/flags.make

CMakeFiles/localization.dir/src/map/localization.o: CMakeFiles/localization.dir/flags.make
CMakeFiles/localization.dir/src/map/localization.o: src/map/localization.cpp
CMakeFiles/localization.dir/src/map/localization.o: manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/actionlib_msgs/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/actionlib/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/nav_msgs/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/localization.dir/src/map/localization.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/olivier/ros_workspace/nao_behavior_tree/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/localization.dir/src/map/localization.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/localization.dir/src/map/localization.o -c /home/olivier/ros_workspace/nao_behavior_tree/src/map/localization.cpp

CMakeFiles/localization.dir/src/map/localization.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/localization.dir/src/map/localization.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/olivier/ros_workspace/nao_behavior_tree/src/map/localization.cpp > CMakeFiles/localization.dir/src/map/localization.i

CMakeFiles/localization.dir/src/map/localization.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/localization.dir/src/map/localization.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/olivier/ros_workspace/nao_behavior_tree/src/map/localization.cpp -o CMakeFiles/localization.dir/src/map/localization.s

CMakeFiles/localization.dir/src/map/localization.o.requires:
.PHONY : CMakeFiles/localization.dir/src/map/localization.o.requires

CMakeFiles/localization.dir/src/map/localization.o.provides: CMakeFiles/localization.dir/src/map/localization.o.requires
	$(MAKE) -f CMakeFiles/localization.dir/build.make CMakeFiles/localization.dir/src/map/localization.o.provides.build
.PHONY : CMakeFiles/localization.dir/src/map/localization.o.provides

CMakeFiles/localization.dir/src/map/localization.o.provides.build: CMakeFiles/localization.dir/src/map/localization.o

# Object files for target localization
localization_OBJECTS = \
"CMakeFiles/localization.dir/src/map/localization.o"

# External object files for target localization
localization_EXTERNAL_OBJECTS =

bin/localization: CMakeFiles/localization.dir/src/map/localization.o
bin/localization: /usr/local/lib/libopencv_calib3d.so.2.4.8
bin/localization: /usr/local/lib/libopencv_contrib.so.2.4.8
bin/localization: /usr/local/lib/libopencv_core.so.2.4.8
bin/localization: /usr/local/lib/libopencv_features2d.so.2.4.8
bin/localization: /usr/local/lib/libopencv_flann.so.2.4.8
bin/localization: /usr/local/lib/libopencv_gpu.so.2.4.8
bin/localization: /usr/local/lib/libopencv_highgui.so.2.4.8
bin/localization: /usr/local/lib/libopencv_imgproc.so.2.4.8
bin/localization: /usr/local/lib/libopencv_legacy.so.2.4.8
bin/localization: /usr/local/lib/libopencv_ml.so.2.4.8
bin/localization: /usr/local/lib/libopencv_nonfree.so.2.4.8
bin/localization: /usr/local/lib/libopencv_objdetect.so.2.4.8
bin/localization: /usr/local/lib/libopencv_photo.so.2.4.8
bin/localization: /usr/local/lib/libopencv_stitching.so.2.4.8
bin/localization: /usr/local/lib/libopencv_ts.a
bin/localization: /usr/local/lib/libopencv_video.so.2.4.8
bin/localization: /usr/local/lib/libopencv_videostab.so.2.4.8
bin/localization: /usr/local/lib/libopencv_videostab.so.2.4.8
bin/localization: /usr/local/lib/libopencv_video.so.2.4.8
bin/localization: /usr/local/lib/libopencv_ts.a
bin/localization: /usr/local/lib/libopencv_superres.so.2.4.8
bin/localization: /usr/local/lib/libopencv_stitching.so.2.4.8
bin/localization: /usr/local/lib/libopencv_photo.so.2.4.8
bin/localization: /usr/local/lib/libopencv_ocl.so.2.4.8
bin/localization: /usr/local/lib/libopencv_objdetect.so.2.4.8
bin/localization: /usr/local/lib/libopencv_nonfree.so.2.4.8
bin/localization: /usr/local/lib/libopencv_ml.so.2.4.8
bin/localization: /usr/local/lib/libopencv_legacy.so.2.4.8
bin/localization: /usr/local/lib/libopencv_imgproc.so.2.4.8
bin/localization: /usr/local/lib/libopencv_highgui.so.2.4.8
bin/localization: /usr/local/lib/libopencv_gpu.so.2.4.8
bin/localization: /usr/local/lib/libopencv_flann.so.2.4.8
bin/localization: /usr/local/lib/libopencv_features2d.so.2.4.8
bin/localization: /usr/local/lib/libopencv_core.so.2.4.8
bin/localization: /usr/local/lib/libopencv_contrib.so.2.4.8
bin/localization: /usr/local/lib/libopencv_calib3d.so.2.4.8
bin/localization: /usr/local/lib/libopencv_nonfree.so.2.4.8
bin/localization: /usr/local/lib/libopencv_gpu.so.2.4.8
bin/localization: /usr/local/lib/libopencv_legacy.so.2.4.8
bin/localization: /usr/local/lib/libopencv_photo.so.2.4.8
bin/localization: /usr/local/lib/libopencv_ocl.so.2.4.8
bin/localization: /usr/local/lib/libopencv_calib3d.so.2.4.8
bin/localization: /usr/local/lib/libopencv_features2d.so.2.4.8
bin/localization: /usr/local/lib/libopencv_flann.so.2.4.8
bin/localization: /usr/local/lib/libopencv_ml.so.2.4.8
bin/localization: /usr/local/lib/libopencv_objdetect.so.2.4.8
bin/localization: /usr/local/lib/libopencv_highgui.so.2.4.8
bin/localization: /usr/local/lib/libopencv_video.so.2.4.8
bin/localization: /usr/local/lib/libopencv_imgproc.so.2.4.8
bin/localization: /usr/local/lib/libopencv_core.so.2.4.8
bin/localization: CMakeFiles/localization.dir/build.make
bin/localization: CMakeFiles/localization.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/localization"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/localization.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/localization.dir/build: bin/localization
.PHONY : CMakeFiles/localization.dir/build

CMakeFiles/localization.dir/requires: CMakeFiles/localization.dir/src/map/localization.o.requires
.PHONY : CMakeFiles/localization.dir/requires

CMakeFiles/localization.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/localization.dir/cmake_clean.cmake
.PHONY : CMakeFiles/localization.dir/clean

CMakeFiles/localization.dir/depend:
	cd /home/olivier/ros_workspace/nao_behavior_tree && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/olivier/ros_workspace/nao_behavior_tree /home/olivier/ros_workspace/nao_behavior_tree /home/olivier/ros_workspace/nao_behavior_tree /home/olivier/ros_workspace/nao_behavior_tree /home/olivier/ros_workspace/nao_behavior_tree/CMakeFiles/localization.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/localization.dir/depend
