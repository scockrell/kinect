# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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
CMAKE_SOURCE_DIR = /home/stephanie/code/dev_stacks/kinect/proximity_detection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/stephanie/code/dev_stacks/kinect/proximity_detection/build

# Include any dependencies generated for this target.
include CMakeFiles/PixelDepthSample.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/PixelDepthSample.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/PixelDepthSample.dir/flags.make

CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: CMakeFiles/PixelDepthSample.dir/flags.make
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: ../src/PixelDepthSample.cpp
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: ../manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/ros/tools/rospack/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/ros/core/roslib/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/ros/core/rosbuild/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/ros/core/roslang/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/clients/rospy/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/utilities/cpp_common/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/utilities/rostime/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/tools/rosconsole/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/openni_kinect/openni/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/openni_kinect/nite/manifest.xml
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/msg_gen/generated
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/msg_gen/generated
CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/stephanie/code/dev_stacks/kinect/proximity_detection/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o -c /home/stephanie/code/dev_stacks/kinect/proximity_detection/src/PixelDepthSample.cpp

CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/stephanie/code/dev_stacks/kinect/proximity_detection/src/PixelDepthSample.cpp > CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.i

CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/stephanie/code/dev_stacks/kinect/proximity_detection/src/PixelDepthSample.cpp -o CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.s

CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o.requires:
.PHONY : CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o.requires

CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o.provides: CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o.requires
	$(MAKE) -f CMakeFiles/PixelDepthSample.dir/build.make CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o.provides.build
.PHONY : CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o.provides

CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o.provides.build: CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o
.PHONY : CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o.provides.build

# Object files for target PixelDepthSample
PixelDepthSample_OBJECTS = \
"CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o"

# External object files for target PixelDepthSample
PixelDepthSample_EXTERNAL_OBJECTS =

../bin/PixelDepthSample: CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o
../bin/PixelDepthSample: CMakeFiles/PixelDepthSample.dir/build.make
../bin/PixelDepthSample: CMakeFiles/PixelDepthSample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/PixelDepthSample"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PixelDepthSample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/PixelDepthSample.dir/build: ../bin/PixelDepthSample
.PHONY : CMakeFiles/PixelDepthSample.dir/build

CMakeFiles/PixelDepthSample.dir/requires: CMakeFiles/PixelDepthSample.dir/src/PixelDepthSample.o.requires
.PHONY : CMakeFiles/PixelDepthSample.dir/requires

CMakeFiles/PixelDepthSample.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/PixelDepthSample.dir/cmake_clean.cmake
.PHONY : CMakeFiles/PixelDepthSample.dir/clean

CMakeFiles/PixelDepthSample.dir/depend:
	cd /home/stephanie/code/dev_stacks/kinect/proximity_detection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/stephanie/code/dev_stacks/kinect/proximity_detection /home/stephanie/code/dev_stacks/kinect/proximity_detection /home/stephanie/code/dev_stacks/kinect/proximity_detection/build /home/stephanie/code/dev_stacks/kinect/proximity_detection/build /home/stephanie/code/dev_stacks/kinect/proximity_detection/build/CMakeFiles/PixelDepthSample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/PixelDepthSample.dir/depend

