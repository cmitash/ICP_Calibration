# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/colin/sandbox/icp_calibration/pcl-tools

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/colin/sandbox/icp_calibration/pcl-tools

# Include any dependencies generated for this target.
include CMakeFiles/normal_estimation_omp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/normal_estimation_omp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/normal_estimation_omp.dir/flags.make

CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o: CMakeFiles/normal_estimation_omp.dir/flags.make
CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o: src/normal_estimation_omp.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/colin/sandbox/icp_calibration/pcl-tools/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o -c /home/colin/sandbox/icp_calibration/pcl-tools/src/normal_estimation_omp.cpp

CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/colin/sandbox/icp_calibration/pcl-tools/src/normal_estimation_omp.cpp > CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.i

CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/colin/sandbox/icp_calibration/pcl-tools/src/normal_estimation_omp.cpp -o CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.s

CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o.requires:
.PHONY : CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o.requires

CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o.provides: CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o.requires
	$(MAKE) -f CMakeFiles/normal_estimation_omp.dir/build.make CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o.provides.build
.PHONY : CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o.provides

CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o.provides.build: CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o

# Object files for target normal_estimation_omp
normal_estimation_omp_OBJECTS = \
"CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o"

# External object files for target normal_estimation_omp
normal_estimation_omp_EXTERNAL_OBJECTS =

normal_estimation_omp: CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o
normal_estimation_omp: CMakeFiles/normal_estimation_omp.dir/build.make
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_system.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libpthread.so
normal_estimation_omp: /usr/local/lib/libpcl_common.so
normal_estimation_omp: /usr/lib/libOpenNI.so
normal_estimation_omp: /usr/lib/libOpenNI2.so
normal_estimation_omp: /usr/lib/libvtkGenericFiltering.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkGeovis.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkCharts.so.5.8.0
normal_estimation_omp: /usr/local/lib/libpcl_io.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_system.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libpthread.so
normal_estimation_omp: /usr/local/lib/libpcl_common.so
normal_estimation_omp: /usr/local/lib/libpcl_octree.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_system.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
normal_estimation_omp: /usr/lib/x86_64-linux-gnu/libpthread.so
normal_estimation_omp: /usr/local/lib/libpcl_common.so
normal_estimation_omp: /usr/local/lib/libpcl_search.so
normal_estimation_omp: /usr/local/lib/libpcl_features.so
normal_estimation_omp: /usr/lib/libOpenNI.so
normal_estimation_omp: /usr/lib/libOpenNI2.so
normal_estimation_omp: /usr/local/lib/libpcl_io.so
normal_estimation_omp: /usr/local/lib/libpcl_octree.so
normal_estimation_omp: /usr/local/lib/libpcl_search.so
normal_estimation_omp: /usr/local/lib/libpcl_features.so
normal_estimation_omp: /usr/lib/libvtkViews.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkInfovis.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkWidgets.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkVolumeRendering.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkHybrid.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkParallel.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkRendering.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkImaging.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkGraphics.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkIO.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkFiltering.so.5.8.0
normal_estimation_omp: /usr/lib/libvtkCommon.so.5.8.0
normal_estimation_omp: /usr/lib/libvtksys.so.5.8.0
normal_estimation_omp: CMakeFiles/normal_estimation_omp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable normal_estimation_omp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/normal_estimation_omp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/normal_estimation_omp.dir/build: normal_estimation_omp
.PHONY : CMakeFiles/normal_estimation_omp.dir/build

CMakeFiles/normal_estimation_omp.dir/requires: CMakeFiles/normal_estimation_omp.dir/src/normal_estimation_omp.cpp.o.requires
.PHONY : CMakeFiles/normal_estimation_omp.dir/requires

CMakeFiles/normal_estimation_omp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/normal_estimation_omp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/normal_estimation_omp.dir/clean

CMakeFiles/normal_estimation_omp.dir/depend:
	cd /home/colin/sandbox/icp_calibration/pcl-tools && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/colin/sandbox/icp_calibration/pcl-tools /home/colin/sandbox/icp_calibration/pcl-tools /home/colin/sandbox/icp_calibration/pcl-tools /home/colin/sandbox/icp_calibration/pcl-tools /home/colin/sandbox/icp_calibration/pcl-tools/CMakeFiles/normal_estimation_omp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/normal_estimation_omp.dir/depend

