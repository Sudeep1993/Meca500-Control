# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/sudeepta/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/sudeepta/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio/build

# Include any dependencies generated for this target.
include CMakeFiles/socket_comm.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/socket_comm.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/socket_comm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/socket_comm.dir/flags.make

CMakeFiles/socket_comm.dir/src/socket_comm.cpp.o: CMakeFiles/socket_comm.dir/flags.make
CMakeFiles/socket_comm.dir/src/socket_comm.cpp.o: /home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio/src/socket_comm.cpp
CMakeFiles/socket_comm.dir/src/socket_comm.cpp.o: CMakeFiles/socket_comm.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/socket_comm.dir/src/socket_comm.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/socket_comm.dir/src/socket_comm.cpp.o -MF CMakeFiles/socket_comm.dir/src/socket_comm.cpp.o.d -o CMakeFiles/socket_comm.dir/src/socket_comm.cpp.o -c /home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio/src/socket_comm.cpp

CMakeFiles/socket_comm.dir/src/socket_comm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/socket_comm.dir/src/socket_comm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio/src/socket_comm.cpp > CMakeFiles/socket_comm.dir/src/socket_comm.cpp.i

CMakeFiles/socket_comm.dir/src/socket_comm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/socket_comm.dir/src/socket_comm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio/src/socket_comm.cpp -o CMakeFiles/socket_comm.dir/src/socket_comm.cpp.s

# Object files for target socket_comm
socket_comm_OBJECTS = \
"CMakeFiles/socket_comm.dir/src/socket_comm.cpp.o"

# External object files for target socket_comm
socket_comm_EXTERNAL_OBJECTS =

libsocket_comm.a: CMakeFiles/socket_comm.dir/src/socket_comm.cpp.o
libsocket_comm.a: CMakeFiles/socket_comm.dir/build.make
libsocket_comm.a: CMakeFiles/socket_comm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libsocket_comm.a"
	$(CMAKE_COMMAND) -P CMakeFiles/socket_comm.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/socket_comm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/socket_comm.dir/build: libsocket_comm.a
.PHONY : CMakeFiles/socket_comm.dir/build

CMakeFiles/socket_comm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/socket_comm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/socket_comm.dir/clean

CMakeFiles/socket_comm.dir/depend:
	cd /home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio /home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio /home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio/build /home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio/build /home/sudeepta/Sudeepta/Experiment_Meca500/Meca500_Pinocchio/build/CMakeFiles/socket_comm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/socket_comm.dir/depend

