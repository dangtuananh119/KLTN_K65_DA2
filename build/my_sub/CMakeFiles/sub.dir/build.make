# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/thuongdx/sub_ar/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/thuongdx/sub_ar/build

# Include any dependencies generated for this target.
include my_sub/CMakeFiles/sub.dir/depend.make

# Include the progress variables for this target.
include my_sub/CMakeFiles/sub.dir/progress.make

# Include the compile flags for this target's objects.
include my_sub/CMakeFiles/sub.dir/flags.make

my_sub/CMakeFiles/sub.dir/src/sub.cpp.o: my_sub/CMakeFiles/sub.dir/flags.make
my_sub/CMakeFiles/sub.dir/src/sub.cpp.o: /home/thuongdx/sub_ar/src/my_sub/src/sub.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/thuongdx/sub_ar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object my_sub/CMakeFiles/sub.dir/src/sub.cpp.o"
	cd /home/thuongdx/sub_ar/build/my_sub && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sub.dir/src/sub.cpp.o -c /home/thuongdx/sub_ar/src/my_sub/src/sub.cpp

my_sub/CMakeFiles/sub.dir/src/sub.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sub.dir/src/sub.cpp.i"
	cd /home/thuongdx/sub_ar/build/my_sub && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/thuongdx/sub_ar/src/my_sub/src/sub.cpp > CMakeFiles/sub.dir/src/sub.cpp.i

my_sub/CMakeFiles/sub.dir/src/sub.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sub.dir/src/sub.cpp.s"
	cd /home/thuongdx/sub_ar/build/my_sub && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/thuongdx/sub_ar/src/my_sub/src/sub.cpp -o CMakeFiles/sub.dir/src/sub.cpp.s

# Object files for target sub
sub_OBJECTS = \
"CMakeFiles/sub.dir/src/sub.cpp.o"

# External object files for target sub
sub_EXTERNAL_OBJECTS =

/home/thuongdx/sub_ar/devel/lib/my_sub/sub: my_sub/CMakeFiles/sub.dir/src/sub.cpp.o
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: my_sub/CMakeFiles/sub.dir/build.make
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /opt/ros/noetic/lib/libroscpp.so
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /opt/ros/noetic/lib/librosconsole.so
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /opt/ros/noetic/lib/librostime.so
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /opt/ros/noetic/lib/libcpp_common.so
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/thuongdx/sub_ar/devel/lib/my_sub/sub: my_sub/CMakeFiles/sub.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/thuongdx/sub_ar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/thuongdx/sub_ar/devel/lib/my_sub/sub"
	cd /home/thuongdx/sub_ar/build/my_sub && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sub.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
my_sub/CMakeFiles/sub.dir/build: /home/thuongdx/sub_ar/devel/lib/my_sub/sub

.PHONY : my_sub/CMakeFiles/sub.dir/build

my_sub/CMakeFiles/sub.dir/clean:
	cd /home/thuongdx/sub_ar/build/my_sub && $(CMAKE_COMMAND) -P CMakeFiles/sub.dir/cmake_clean.cmake
.PHONY : my_sub/CMakeFiles/sub.dir/clean

my_sub/CMakeFiles/sub.dir/depend:
	cd /home/thuongdx/sub_ar/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/thuongdx/sub_ar/src /home/thuongdx/sub_ar/src/my_sub /home/thuongdx/sub_ar/build /home/thuongdx/sub_ar/build/my_sub /home/thuongdx/sub_ar/build/my_sub/CMakeFiles/sub.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : my_sub/CMakeFiles/sub.dir/depend

