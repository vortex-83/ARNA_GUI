# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/vogel/Code/LARRI/ARNA_GUI

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vogel/Code/LARRI/ARNA_GUI

# Include any dependencies generated for this target.
include CMakeFiles/ARNA_GUI2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ARNA_GUI2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ARNA_GUI2.dir/flags.make

CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o: CMakeFiles/ARNA_GUI2.dir/flags.make
CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o: src/ARNA_theora.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vogel/Code/LARRI/ARNA_GUI/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o -c /home/vogel/Code/LARRI/ARNA_GUI/src/ARNA_theora.cpp

CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vogel/Code/LARRI/ARNA_GUI/src/ARNA_theora.cpp > CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.i

CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vogel/Code/LARRI/ARNA_GUI/src/ARNA_theora.cpp -o CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.s

CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o.requires:

.PHONY : CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o.requires

CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o.provides: CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o.requires
	$(MAKE) -f CMakeFiles/ARNA_GUI2.dir/build.make CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o.provides.build
.PHONY : CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o.provides

CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o.provides.build: CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o


# Object files for target ARNA_GUI2
ARNA_GUI2_OBJECTS = \
"CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o"

# External object files for target ARNA_GUI2
ARNA_GUI2_EXTERNAL_OBJECTS =

ARNA_GUI2: CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o
ARNA_GUI2: CMakeFiles/ARNA_GUI2.dir/build.make
ARNA_GUI2: CMakeFiles/ARNA_GUI2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vogel/Code/LARRI/ARNA_GUI/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ARNA_GUI2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ARNA_GUI2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ARNA_GUI2.dir/build: ARNA_GUI2

.PHONY : CMakeFiles/ARNA_GUI2.dir/build

CMakeFiles/ARNA_GUI2.dir/requires: CMakeFiles/ARNA_GUI2.dir/src/ARNA_theora.cpp.o.requires

.PHONY : CMakeFiles/ARNA_GUI2.dir/requires

CMakeFiles/ARNA_GUI2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ARNA_GUI2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ARNA_GUI2.dir/clean

CMakeFiles/ARNA_GUI2.dir/depend:
	cd /home/vogel/Code/LARRI/ARNA_GUI && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vogel/Code/LARRI/ARNA_GUI /home/vogel/Code/LARRI/ARNA_GUI /home/vogel/Code/LARRI/ARNA_GUI /home/vogel/Code/LARRI/ARNA_GUI /home/vogel/Code/LARRI/ARNA_GUI/CMakeFiles/ARNA_GUI2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ARNA_GUI2.dir/depend

