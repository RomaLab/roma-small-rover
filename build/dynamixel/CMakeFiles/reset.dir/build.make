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
CMAKE_SOURCE_DIR = /home/roma/roma-small-rover/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/roma/roma-small-rover/build

# Include any dependencies generated for this target.
include dynamixel/CMakeFiles/reset.dir/depend.make

# Include the progress variables for this target.
include dynamixel/CMakeFiles/reset.dir/progress.make

# Include the compile flags for this target's objects.
include dynamixel/CMakeFiles/reset.dir/flags.make

dynamixel/CMakeFiles/reset.dir/src/reset.cpp.o: dynamixel/CMakeFiles/reset.dir/flags.make
dynamixel/CMakeFiles/reset.dir/src/reset.cpp.o: /home/roma/roma-small-rover/src/dynamixel/src/reset.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/roma/roma-small-rover/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object dynamixel/CMakeFiles/reset.dir/src/reset.cpp.o"
	cd /home/roma/roma-small-rover/build/dynamixel && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/reset.dir/src/reset.cpp.o -c /home/roma/roma-small-rover/src/dynamixel/src/reset.cpp

dynamixel/CMakeFiles/reset.dir/src/reset.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/reset.dir/src/reset.cpp.i"
	cd /home/roma/roma-small-rover/build/dynamixel && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/roma/roma-small-rover/src/dynamixel/src/reset.cpp > CMakeFiles/reset.dir/src/reset.cpp.i

dynamixel/CMakeFiles/reset.dir/src/reset.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/reset.dir/src/reset.cpp.s"
	cd /home/roma/roma-small-rover/build/dynamixel && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/roma/roma-small-rover/src/dynamixel/src/reset.cpp -o CMakeFiles/reset.dir/src/reset.cpp.s

# Object files for target reset
reset_OBJECTS = \
"CMakeFiles/reset.dir/src/reset.cpp.o"

# External object files for target reset
reset_EXTERNAL_OBJECTS =

/home/roma/roma-small-rover/devel/lib/dynamixel/reset: dynamixel/CMakeFiles/reset.dir/src/reset.cpp.o
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: dynamixel/CMakeFiles/reset.dir/build.make
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /home/roma/roma-small-rover/devel/lib/libdynamixel_sdk.so
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /opt/ros/noetic/lib/libroscpp.so
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /opt/ros/noetic/lib/librosconsole.so
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /opt/ros/noetic/lib/librostime.so
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /opt/ros/noetic/lib/libcpp_common.so
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/roma/roma-small-rover/devel/lib/dynamixel/reset: dynamixel/CMakeFiles/reset.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/roma/roma-small-rover/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/roma/roma-small-rover/devel/lib/dynamixel/reset"
	cd /home/roma/roma-small-rover/build/dynamixel && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/reset.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dynamixel/CMakeFiles/reset.dir/build: /home/roma/roma-small-rover/devel/lib/dynamixel/reset

.PHONY : dynamixel/CMakeFiles/reset.dir/build

dynamixel/CMakeFiles/reset.dir/clean:
	cd /home/roma/roma-small-rover/build/dynamixel && $(CMAKE_COMMAND) -P CMakeFiles/reset.dir/cmake_clean.cmake
.PHONY : dynamixel/CMakeFiles/reset.dir/clean

dynamixel/CMakeFiles/reset.dir/depend:
	cd /home/roma/roma-small-rover/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/roma/roma-small-rover/src /home/roma/roma-small-rover/src/dynamixel /home/roma/roma-small-rover/build /home/roma/roma-small-rover/build/dynamixel /home/roma/roma-small-rover/build/dynamixel/CMakeFiles/reset.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dynamixel/CMakeFiles/reset.dir/depend

