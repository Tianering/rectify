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
CMAKE_SOURCE_DIR = /home/shanyi/WorkSpace/rectify

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shanyi/WorkSpace/rectify/build

# Include any dependencies generated for this target.
include CMakeFiles/rectify.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rectify.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rectify.dir/flags.make

CMakeFiles/rectify.dir/main.cpp.o: CMakeFiles/rectify.dir/flags.make
CMakeFiles/rectify.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shanyi/WorkSpace/rectify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rectify.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rectify.dir/main.cpp.o -c /home/shanyi/WorkSpace/rectify/main.cpp

CMakeFiles/rectify.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rectify.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shanyi/WorkSpace/rectify/main.cpp > CMakeFiles/rectify.dir/main.cpp.i

CMakeFiles/rectify.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rectify.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shanyi/WorkSpace/rectify/main.cpp -o CMakeFiles/rectify.dir/main.cpp.s

# Object files for target rectify
rectify_OBJECTS = \
"CMakeFiles/rectify.dir/main.cpp.o"

# External object files for target rectify
rectify_EXTERNAL_OBJECTS =

rectify: CMakeFiles/rectify.dir/main.cpp.o
rectify: CMakeFiles/rectify.dir/build.make
rectify: CMakeFiles/rectify.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shanyi/WorkSpace/rectify/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rectify"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rectify.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rectify.dir/build: rectify

.PHONY : CMakeFiles/rectify.dir/build

CMakeFiles/rectify.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rectify.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rectify.dir/clean

CMakeFiles/rectify.dir/depend:
	cd /home/shanyi/WorkSpace/rectify/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shanyi/WorkSpace/rectify /home/shanyi/WorkSpace/rectify /home/shanyi/WorkSpace/rectify/build /home/shanyi/WorkSpace/rectify/build /home/shanyi/WorkSpace/rectify/build/CMakeFiles/rectify.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rectify.dir/depend

