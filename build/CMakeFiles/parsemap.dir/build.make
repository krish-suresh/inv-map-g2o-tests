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
CMAKE_SOURCE_DIR = /home/ksuresh/inv-map-work-22/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ksuresh/inv-map-work-22/build

# Include any dependencies generated for this target.
include CMakeFiles/parsemap.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/parsemap.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/parsemap.dir/flags.make

CMakeFiles/parsemap.dir/parsemap.cpp.o: CMakeFiles/parsemap.dir/flags.make
CMakeFiles/parsemap.dir/parsemap.cpp.o: /home/ksuresh/inv-map-work-22/src/parsemap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ksuresh/inv-map-work-22/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/parsemap.dir/parsemap.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/parsemap.dir/parsemap.cpp.o -c /home/ksuresh/inv-map-work-22/src/parsemap.cpp

CMakeFiles/parsemap.dir/parsemap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/parsemap.dir/parsemap.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ksuresh/inv-map-work-22/src/parsemap.cpp > CMakeFiles/parsemap.dir/parsemap.cpp.i

CMakeFiles/parsemap.dir/parsemap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/parsemap.dir/parsemap.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ksuresh/inv-map-work-22/src/parsemap.cpp -o CMakeFiles/parsemap.dir/parsemap.cpp.s

# Object files for target parsemap
parsemap_OBJECTS = \
"CMakeFiles/parsemap.dir/parsemap.cpp.o"

# External object files for target parsemap
parsemap_EXTERNAL_OBJECTS =

parsemap: CMakeFiles/parsemap.dir/parsemap.cpp.o
parsemap: CMakeFiles/parsemap.dir/build.make
parsemap: CMakeFiles/parsemap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ksuresh/inv-map-work-22/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable parsemap"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/parsemap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/parsemap.dir/build: parsemap

.PHONY : CMakeFiles/parsemap.dir/build

CMakeFiles/parsemap.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/parsemap.dir/cmake_clean.cmake
.PHONY : CMakeFiles/parsemap.dir/clean

CMakeFiles/parsemap.dir/depend:
	cd /home/ksuresh/inv-map-work-22/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ksuresh/inv-map-work-22/src /home/ksuresh/inv-map-work-22/src /home/ksuresh/inv-map-work-22/build /home/ksuresh/inv-map-work-22/build /home/ksuresh/inv-map-work-22/build/CMakeFiles/parsemap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/parsemap.dir/depend
