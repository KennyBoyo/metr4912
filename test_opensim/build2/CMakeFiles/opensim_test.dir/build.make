# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kenzo/documents/metr4912/test_opensim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kenzo/documents/metr4912/test_opensim/build2

# Include any dependencies generated for this target.
include CMakeFiles/opensim_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/opensim_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/opensim_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/opensim_test.dir/flags.make

CMakeFiles/opensim_test.dir/src/opensim-test.cpp.o: CMakeFiles/opensim_test.dir/flags.make
CMakeFiles/opensim_test.dir/src/opensim-test.cpp.o: ../src/opensim-test.cpp
CMakeFiles/opensim_test.dir/src/opensim-test.cpp.o: CMakeFiles/opensim_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kenzo/documents/metr4912/test_opensim/build2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/opensim_test.dir/src/opensim-test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/opensim_test.dir/src/opensim-test.cpp.o -MF CMakeFiles/opensim_test.dir/src/opensim-test.cpp.o.d -o CMakeFiles/opensim_test.dir/src/opensim-test.cpp.o -c /home/kenzo/documents/metr4912/test_opensim/src/opensim-test.cpp

CMakeFiles/opensim_test.dir/src/opensim-test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opensim_test.dir/src/opensim-test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kenzo/documents/metr4912/test_opensim/src/opensim-test.cpp > CMakeFiles/opensim_test.dir/src/opensim-test.cpp.i

CMakeFiles/opensim_test.dir/src/opensim-test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opensim_test.dir/src/opensim-test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kenzo/documents/metr4912/test_opensim/src/opensim-test.cpp -o CMakeFiles/opensim_test.dir/src/opensim-test.cpp.s

# Object files for target opensim_test
opensim_test_OBJECTS = \
"CMakeFiles/opensim_test.dir/src/opensim-test.cpp.o"

# External object files for target opensim_test
opensim_test_EXTERNAL_OBJECTS =

opensim_test: CMakeFiles/opensim_test.dir/src/opensim-test.cpp.o
opensim_test: CMakeFiles/opensim_test.dir/build.make
opensim_test: /home/kenzo/opensim-core/sdk/lib/libosimExampleComponents.so
opensim_test: /home/kenzo/opensim-core/sdk/lib/libosimMoco.so
opensim_test: /home/kenzo/opensim-core/sdk/lib/libosimTools.so
opensim_test: /home/kenzo/opensim-core/sdk/lib/libosimAnalyses.so
opensim_test: /home/kenzo/opensim-core/sdk/lib/libosimActuators.so
opensim_test: /home/kenzo/opensim-core/sdk/lib/libosimSimulation.so
opensim_test: /home/kenzo/opensim-core/sdk/lib/libosimCommon.so
opensim_test: /home/kenzo/opensim-core/sdk/Simbody/lib/libSimTKsimbody.so.3.8
opensim_test: /home/kenzo/opensim-core/sdk/Simbody/lib/libSimTKmath.so.3.8
opensim_test: /home/kenzo/opensim-core/sdk/Simbody/lib/libSimTKcommon.so.3.8
opensim_test: /usr/lib/x86_64-linux-gnu/libopenblas.so
opensim_test: /home/kenzo/opensim-core/sdk/spdlog/lib/spdlog/libspdlog.a
opensim_test: /home/kenzo/opensim-core/sdk/lib/libosimLepton.so
opensim_test: CMakeFiles/opensim_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kenzo/documents/metr4912/test_opensim/build2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable opensim_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opensim_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/opensim_test.dir/build: opensim_test
.PHONY : CMakeFiles/opensim_test.dir/build

CMakeFiles/opensim_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/opensim_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/opensim_test.dir/clean

CMakeFiles/opensim_test.dir/depend:
	cd /home/kenzo/documents/metr4912/test_opensim/build2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kenzo/documents/metr4912/test_opensim /home/kenzo/documents/metr4912/test_opensim /home/kenzo/documents/metr4912/test_opensim/build2 /home/kenzo/documents/metr4912/test_opensim/build2 /home/kenzo/documents/metr4912/test_opensim/build2/CMakeFiles/opensim_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/opensim_test.dir/depend
