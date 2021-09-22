cmake_minimum_required(VERSION 3.17)
project(openMVGLite)

set(CMAKE_CXX_STANDARD 17)

add_executable(openMVGLite sfm/main_SfMInit_ImageListing.cpp sfm/main_ComputeFeatures.cpp)
#add_executable(main_feature sfm/main_SfMInit_ImageListing.cpp sfm/main_ComputeFeatures.cpp)

find_package(OpenCV REQUIRED)
find_package(OpenMVG REQUIRED)
find_package(Ceres REQUIRED )
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OPENMVG_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})

target_link_libraries(openMVGLite
        PRIVATE
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_camera
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_features
        OpenMVG::openMVG_geodesy
        OpenMVG::openMVG_geometry
        OpenMVG::openMVG_graph
        OpenMVG::openMVG_image
        OpenMVG::openMVG_linearProgramming
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_matching_image_collection
        OpenMVG::openMVG_multiview
        OpenMVG::openMVG_numeric
        OpenMVG::openMVG_robust_estimation
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_system
        ${OpenCV_LIBS}
        )
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()