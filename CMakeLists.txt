cmake_minimum_required(VERSION 3.17)
project(openMVGLite)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(OpenMVG REQUIRED)
find_package(Ceres REQUIRED )

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${OPENMVG_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


add_executable(main_SfMInit_ImageListing sfm/main_SfMInit_ImageListing.cpp)
target_link_libraries(main_SfMInit_ImageListing
        PRIVATE
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_camera
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_image
        ${OpenCV_LIBS}
        )

add_executable(main_ComputeFeatures sfm/main_ComputeFeatures.cpp)
target_link_libraries(main_ComputeFeatures
        PRIVATE
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_features
        OpenMVG::openMVG_image
        ${OpenCV_LIBS}
        )

add_executable(main_ComputeMatches sfm/main_ComputeMatches.cpp)
target_link_libraries(main_ComputeMatches
        PRIVATE
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_robust_estimation
        OpenMVG::openMVG_matching_image_collection
)

add_executable(main_GlobalSfM sfm/main_GlobalSfM.cpp)
target_link_libraries(main_GlobalSfM
        PRIVATE
        OpenMVG::openMVG_sfm
        )
