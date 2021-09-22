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
CMAKE_COMMAND = /opt/clion-2020.3.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2020.3.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/openMVGLite.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/openMVGLite.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/openMVGLite.dir/flags.make

CMakeFiles/openMVGLite.dir/main.cpp.o: CMakeFiles/openMVGLite.dir/flags.make
CMakeFiles/openMVGLite.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/openMVGLite.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/openMVGLite.dir/main.cpp.o -c /home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite/main.cpp

CMakeFiles/openMVGLite.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/openMVGLite.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite/main.cpp > CMakeFiles/openMVGLite.dir/main.cpp.i

CMakeFiles/openMVGLite.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/openMVGLite.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite/main.cpp -o CMakeFiles/openMVGLite.dir/main.cpp.s

# Object files for target openMVGLite
openMVGLite_OBJECTS = \
"CMakeFiles/openMVGLite.dir/main.cpp.o"

# External object files for target openMVGLite
openMVGLite_EXTERNAL_OBJECTS =

openMVGLite: CMakeFiles/openMVGLite.dir/main.cpp.o
openMVGLite: CMakeFiles/openMVGLite.dir/build.make
openMVGLite: /usr/local/lib/libopenMVG_sfm.a
openMVGLite: /usr/local/lib/libopenMVG_matching.a
openMVGLite: /usr/local/lib/libopenMVG_exif.a
openMVGLite: /usr/local/lib/libopenMVG_features.a
openMVGLite: /usr/local/lib/libopenMVG_geometry.a
openMVGLite: /usr/local/lib/libopenMVG_image.a
openMVGLite: /usr/local/lib/libopenMVG_linearProgramming.a
openMVGLite: /usr/local/lib/libopenMVG_matching.a
openMVGLite: /usr/local/lib/libopenMVG_matching_image_collection.a
openMVGLite: /usr/local/lib/libopenMVG_multiview.a
openMVGLite: /usr/local/lib/libopenMVG_numeric.a
openMVGLite: /usr/local/lib/libopenMVG_robust_estimation.a
openMVGLite: /usr/local/lib/libopenMVG_sfm.a
openMVGLite: /usr/local/lib/libopenMVG_system.a
openMVGLite: /usr/local/lib/libopencv_gapi.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_highgui.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_ml.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_objdetect.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_photo.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_stitching.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_video.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_videoio.so.4.5.3
openMVGLite: /usr/local/lib/libopenMVG_geometry.a
openMVGLite: /usr/local/lib/libopenMVG_image.a
openMVGLite: /usr/lib/x86_64-linux-gnu/libjpeg.so
openMVGLite: /usr/lib/x86_64-linux-gnu/libpng.so
openMVGLite: /usr/lib/x86_64-linux-gnu/libz.so
openMVGLite: /usr/lib/x86_64-linux-gnu/libtiff.so
openMVGLite: /usr/local/lib/libopenMVG_lInftyComputerVision.a
openMVGLite: /usr/local/lib/libopenMVG_linearProgramming.a
openMVGLite: /usr/local/lib/liblib_clp.a
openMVGLite: /usr/local/lib/liblib_OsiClpSolver.a
openMVGLite: /usr/local/lib/liblib_CoinUtils.a
openMVGLite: /usr/local/lib/liblib_Osi.a
openMVGLite: /usr/local/lib/libopenMVG_easyexif.a
openMVGLite: /usr/local/lib/libopenMVG_matching.a
openMVGLite: /usr/local/lib/libopenMVG_features.a
openMVGLite: /usr/local/lib/libopenMVG_fast.a
openMVGLite: /usr/local/lib/libopenMVG_stlplus.a
openMVGLite: /usr/local/lib/libopenMVG_multiview.a
openMVGLite: /usr/local/lib/libopenMVG_lemon.a
openMVGLite: /usr/local/lib/libopenMVG_ceres.a
openMVGLite: /usr/local/lib/libopenMVG_cxsparse.a
openMVGLite: /usr/lib/x86_64-linux-gnu/liblapack.so
openMVGLite: /usr/lib/x86_64-linux-gnu/libf77blas.so
openMVGLite: /usr/lib/x86_64-linux-gnu/libatlas.so
openMVGLite: /usr/lib/x86_64-linux-gnu/libf77blas.so
openMVGLite: /usr/lib/x86_64-linux-gnu/libatlas.so
openMVGLite: /usr/local/lib/libopenMVG_numeric.a
openMVGLite: /usr/local/lib/libopencv_dnn.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_imgcodecs.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_calib3d.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_features2d.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_flann.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_imgproc.so.4.5.3
openMVGLite: /usr/local/lib/libopencv_core.so.4.5.3
openMVGLite: CMakeFiles/openMVGLite.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable openMVGLite"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/openMVGLite.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/openMVGLite.dir/build: openMVGLite

.PHONY : CMakeFiles/openMVGLite.dir/build

CMakeFiles/openMVGLite.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/openMVGLite.dir/cmake_clean.cmake
.PHONY : CMakeFiles/openMVGLite.dir/clean

CMakeFiles/openMVGLite.dir/depend:
	cd /home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite /home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite /home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite/cmake-build-debug /home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite/cmake-build-debug /home/mitom/3DReconstruction/MVG_MVS/git/openMVGLite/cmake-build-debug/CMakeFiles/openMVGLite.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/openMVGLite.dir/depend
