# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_COMMAND = /snap/clion/138/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/138/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/filip/banana_ws/src/xbox_controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/filip/banana_ws/src/xbox_controller/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/xbox_controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/xbox_controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/xbox_controller.dir/flags.make

CMakeFiles/xbox_controller.dir/src/xbox_controller_node.cpp.o: CMakeFiles/xbox_controller.dir/flags.make
CMakeFiles/xbox_controller.dir/src/xbox_controller_node.cpp.o: ../src/xbox_controller_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/filip/banana_ws/src/xbox_controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/xbox_controller.dir/src/xbox_controller_node.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xbox_controller.dir/src/xbox_controller_node.cpp.o -c /home/filip/banana_ws/src/xbox_controller/src/xbox_controller_node.cpp

CMakeFiles/xbox_controller.dir/src/xbox_controller_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xbox_controller.dir/src/xbox_controller_node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/filip/banana_ws/src/xbox_controller/src/xbox_controller_node.cpp > CMakeFiles/xbox_controller.dir/src/xbox_controller_node.cpp.i

CMakeFiles/xbox_controller.dir/src/xbox_controller_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xbox_controller.dir/src/xbox_controller_node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/filip/banana_ws/src/xbox_controller/src/xbox_controller_node.cpp -o CMakeFiles/xbox_controller.dir/src/xbox_controller_node.cpp.s

CMakeFiles/xbox_controller.dir/src/XboxController.cpp.o: CMakeFiles/xbox_controller.dir/flags.make
CMakeFiles/xbox_controller.dir/src/XboxController.cpp.o: ../src/XboxController.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/filip/banana_ws/src/xbox_controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/xbox_controller.dir/src/XboxController.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xbox_controller.dir/src/XboxController.cpp.o -c /home/filip/banana_ws/src/xbox_controller/src/XboxController.cpp

CMakeFiles/xbox_controller.dir/src/XboxController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xbox_controller.dir/src/XboxController.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/filip/banana_ws/src/xbox_controller/src/XboxController.cpp > CMakeFiles/xbox_controller.dir/src/XboxController.cpp.i

CMakeFiles/xbox_controller.dir/src/XboxController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xbox_controller.dir/src/XboxController.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/filip/banana_ws/src/xbox_controller/src/XboxController.cpp -o CMakeFiles/xbox_controller.dir/src/XboxController.cpp.s

# Object files for target xbox_controller
xbox_controller_OBJECTS = \
"CMakeFiles/xbox_controller.dir/src/xbox_controller_node.cpp.o" \
"CMakeFiles/xbox_controller.dir/src/XboxController.cpp.o"

# External object files for target xbox_controller
xbox_controller_EXTERNAL_OBJECTS =

devel/lib/xbox_controller/xbox_controller: CMakeFiles/xbox_controller.dir/src/xbox_controller_node.cpp.o
devel/lib/xbox_controller/xbox_controller: CMakeFiles/xbox_controller.dir/src/XboxController.cpp.o
devel/lib/xbox_controller/xbox_controller: CMakeFiles/xbox_controller.dir/build.make
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/libcv_bridge.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/libimage_transport.so
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/libmessage_filters.so
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/libclass_loader.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/libPocoFoundation.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/libroscpp.so
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/librosconsole.so
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/librosconsole_log4cxx.so
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/librosconsole_backend_interface.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/libxmlrpcpp.so
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/libroslib.so
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/librospack.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libpython2.7.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/libroscpp_serialization.so
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/librostime.so
devel/lib/xbox_controller/xbox_controller: /opt/ros/melodic/lib/libcpp_common.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
devel/lib/xbox_controller/xbox_controller: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
devel/lib/xbox_controller/xbox_controller: CMakeFiles/xbox_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/filip/banana_ws/src/xbox_controller/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable devel/lib/xbox_controller/xbox_controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xbox_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/xbox_controller.dir/build: devel/lib/xbox_controller/xbox_controller

.PHONY : CMakeFiles/xbox_controller.dir/build

CMakeFiles/xbox_controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/xbox_controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/xbox_controller.dir/clean

CMakeFiles/xbox_controller.dir/depend:
	cd /home/filip/banana_ws/src/xbox_controller/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/filip/banana_ws/src/xbox_controller /home/filip/banana_ws/src/xbox_controller /home/filip/banana_ws/src/xbox_controller/cmake-build-debug /home/filip/banana_ws/src/xbox_controller/cmake-build-debug /home/filip/banana_ws/src/xbox_controller/cmake-build-debug/CMakeFiles/xbox_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/xbox_controller.dir/depend

