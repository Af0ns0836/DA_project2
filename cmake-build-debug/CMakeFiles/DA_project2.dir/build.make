# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.23

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "D:\Program Files\JetBrains\CLion 2022.2.3\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "D:\Program Files\JetBrains\CLion 2022.2.3\bin\cmake\win\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/DA_project2.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/DA_project2.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/DA_project2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/DA_project2.dir/flags.make

CMakeFiles/DA_project2.dir/main.cpp.obj: CMakeFiles/DA_project2.dir/flags.make
CMakeFiles/DA_project2.dir/main.cpp.obj: CMakeFiles/DA_project2.dir/includes_CXX.rsp
CMakeFiles/DA_project2.dir/main.cpp.obj: ../main.cpp
CMakeFiles/DA_project2.dir/main.cpp.obj: CMakeFiles/DA_project2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\cmake-build-debug\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/DA_project2.dir/main.cpp.obj"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DA_project2.dir/main.cpp.obj -MF CMakeFiles\DA_project2.dir\main.cpp.obj.d -o CMakeFiles\DA_project2.dir\main.cpp.obj -c "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\main.cpp"

CMakeFiles/DA_project2.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DA_project2.dir/main.cpp.i"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\main.cpp" > CMakeFiles\DA_project2.dir\main.cpp.i

CMakeFiles/DA_project2.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DA_project2.dir/main.cpp.s"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\main.cpp" -o CMakeFiles\DA_project2.dir\main.cpp.s

CMakeFiles/DA_project2.dir/Graph.cpp.obj: CMakeFiles/DA_project2.dir/flags.make
CMakeFiles/DA_project2.dir/Graph.cpp.obj: CMakeFiles/DA_project2.dir/includes_CXX.rsp
CMakeFiles/DA_project2.dir/Graph.cpp.obj: ../Graph.cpp
CMakeFiles/DA_project2.dir/Graph.cpp.obj: CMakeFiles/DA_project2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\cmake-build-debug\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/DA_project2.dir/Graph.cpp.obj"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DA_project2.dir/Graph.cpp.obj -MF CMakeFiles\DA_project2.dir\Graph.cpp.obj.d -o CMakeFiles\DA_project2.dir\Graph.cpp.obj -c "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\Graph.cpp"

CMakeFiles/DA_project2.dir/Graph.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DA_project2.dir/Graph.cpp.i"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\Graph.cpp" > CMakeFiles\DA_project2.dir\Graph.cpp.i

CMakeFiles/DA_project2.dir/Graph.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DA_project2.dir/Graph.cpp.s"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\Graph.cpp" -o CMakeFiles\DA_project2.dir\Graph.cpp.s

CMakeFiles/DA_project2.dir/menu.cpp.obj: CMakeFiles/DA_project2.dir/flags.make
CMakeFiles/DA_project2.dir/menu.cpp.obj: CMakeFiles/DA_project2.dir/includes_CXX.rsp
CMakeFiles/DA_project2.dir/menu.cpp.obj: ../menu.cpp
CMakeFiles/DA_project2.dir/menu.cpp.obj: CMakeFiles/DA_project2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\cmake-build-debug\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/DA_project2.dir/menu.cpp.obj"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DA_project2.dir/menu.cpp.obj -MF CMakeFiles\DA_project2.dir\menu.cpp.obj.d -o CMakeFiles\DA_project2.dir\menu.cpp.obj -c "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\menu.cpp"

CMakeFiles/DA_project2.dir/menu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DA_project2.dir/menu.cpp.i"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\menu.cpp" > CMakeFiles\DA_project2.dir\menu.cpp.i

CMakeFiles/DA_project2.dir/menu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DA_project2.dir/menu.cpp.s"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\menu.cpp" -o CMakeFiles\DA_project2.dir\menu.cpp.s

CMakeFiles/DA_project2.dir/TSP.cpp.obj: CMakeFiles/DA_project2.dir/flags.make
CMakeFiles/DA_project2.dir/TSP.cpp.obj: CMakeFiles/DA_project2.dir/includes_CXX.rsp
CMakeFiles/DA_project2.dir/TSP.cpp.obj: ../TSP.cpp
CMakeFiles/DA_project2.dir/TSP.cpp.obj: CMakeFiles/DA_project2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\cmake-build-debug\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/DA_project2.dir/TSP.cpp.obj"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DA_project2.dir/TSP.cpp.obj -MF CMakeFiles\DA_project2.dir\TSP.cpp.obj.d -o CMakeFiles\DA_project2.dir\TSP.cpp.obj -c "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\TSP.cpp"

CMakeFiles/DA_project2.dir/TSP.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DA_project2.dir/TSP.cpp.i"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\TSP.cpp" > CMakeFiles\DA_project2.dir\TSP.cpp.i

CMakeFiles/DA_project2.dir/TSP.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DA_project2.dir/TSP.cpp.s"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\TSP.cpp" -o CMakeFiles\DA_project2.dir\TSP.cpp.s

CMakeFiles/DA_project2.dir/VertexEdge.cpp.obj: CMakeFiles/DA_project2.dir/flags.make
CMakeFiles/DA_project2.dir/VertexEdge.cpp.obj: CMakeFiles/DA_project2.dir/includes_CXX.rsp
CMakeFiles/DA_project2.dir/VertexEdge.cpp.obj: ../VertexEdge.cpp
CMakeFiles/DA_project2.dir/VertexEdge.cpp.obj: CMakeFiles/DA_project2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\cmake-build-debug\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/DA_project2.dir/VertexEdge.cpp.obj"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/DA_project2.dir/VertexEdge.cpp.obj -MF CMakeFiles\DA_project2.dir\VertexEdge.cpp.obj.d -o CMakeFiles\DA_project2.dir\VertexEdge.cpp.obj -c "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\VertexEdge.cpp"

CMakeFiles/DA_project2.dir/VertexEdge.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DA_project2.dir/VertexEdge.cpp.i"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\VertexEdge.cpp" > CMakeFiles\DA_project2.dir\VertexEdge.cpp.i

CMakeFiles/DA_project2.dir/VertexEdge.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DA_project2.dir/VertexEdge.cpp.s"
	"D:\Program Files\JetBrains\CLion 2022.2.3\bin\mingw\bin\g++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\VertexEdge.cpp" -o CMakeFiles\DA_project2.dir\VertexEdge.cpp.s

# Object files for target DA_project2
DA_project2_OBJECTS = \
"CMakeFiles/DA_project2.dir/main.cpp.obj" \
"CMakeFiles/DA_project2.dir/Graph.cpp.obj" \
"CMakeFiles/DA_project2.dir/menu.cpp.obj" \
"CMakeFiles/DA_project2.dir/TSP.cpp.obj" \
"CMakeFiles/DA_project2.dir/VertexEdge.cpp.obj"

# External object files for target DA_project2
DA_project2_EXTERNAL_OBJECTS =

DA_project2.exe: CMakeFiles/DA_project2.dir/main.cpp.obj
DA_project2.exe: CMakeFiles/DA_project2.dir/Graph.cpp.obj
DA_project2.exe: CMakeFiles/DA_project2.dir/menu.cpp.obj
DA_project2.exe: CMakeFiles/DA_project2.dir/TSP.cpp.obj
DA_project2.exe: CMakeFiles/DA_project2.dir/VertexEdge.cpp.obj
DA_project2.exe: CMakeFiles/DA_project2.dir/build.make
DA_project2.exe: CMakeFiles/DA_project2.dir/linklibs.rsp
DA_project2.exe: CMakeFiles/DA_project2.dir/objects1.rsp
DA_project2.exe: CMakeFiles/DA_project2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\cmake-build-debug\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable DA_project2.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\DA_project2.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/DA_project2.dir/build: DA_project2.exe
.PHONY : CMakeFiles/DA_project2.dir/build

CMakeFiles/DA_project2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\DA_project2.dir\cmake_clean.cmake
.PHONY : CMakeFiles/DA_project2.dir/clean

CMakeFiles/DA_project2.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main" "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main" "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\cmake-build-debug" "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\cmake-build-debug" "C:\Users\35191\Desktop\DA_project2-main (5) - Copy\DA_project2-main\cmake-build-debug\CMakeFiles\DA_project2.dir\DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/DA_project2.dir/depend

