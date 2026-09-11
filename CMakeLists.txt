cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

message("Build type: " ${CMAKE_BUILD_TYPE})
option(BUILD_PYTHON_BINDINGS "Build Python bindings for ORB_SLAM2" ON)
message(STATUS "Build Python bindings: ${BUILD_PYTHON_BINDINGS}")

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3  ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 ")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
set(CMAKE_CXX_FLAGS "-std=c++17 -O2")

# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#    add_definitions(-DCOMPILEDWITHC11)
#    message(STATUS "Using flag -std=c++11.")
# elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#    add_definitions(-DCOMPILEDWITHC0X)
#    message(STATUS "Using flag -std=c++0x.")
# else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.11 QUIET)
# if(NOT OpenCV_FOUND)
#    find_package(OpenCV 2.4.3 QUIET)
#    if(NOT OpenCV_FOUND)
#       message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#    endif()
# endif()

# find_package(Eigen3 3.4.0 REQUIRED)
# find_package(Pangolin REQUIRED)
find_package(Eigen3 3.4 REQUIRED CONFIG NO_MODULE)
set(Pangolin_DIR "/home/lzm/Project2/ORB_SLAM2/Thirdparty/Pangolin/build")
find_package(Pangolin REQUIRED CONFIG)


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/DynamicDetection.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

# add_executable(rgbd_tum
# Examples/RGB-D/rgbd_tum.cc)
# target_link_libraries(rgbd_tum ${PROJECT_NAME})

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

# add_executable(stereo_kitti
# Examples/Stereo/stereo_kitti.cc)
# target_link_libraries(stereo_kitti ${PROJECT_NAME})

# add_executable(stereo_euroc
# Examples/Stereo/stereo_euroc.cc)
# target_link_libraries(stereo_euroc ${PROJECT_NAME})


# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

# add_executable(mono_tum
# Examples/Monocular/mono_tum.cc)
# target_link_libraries(mono_tum ${PROJECT_NAME})

# add_executable(mono_kitti
# Examples/Monocular/mono_kitti.cc)
# target_link_libraries(mono_kitti ${PROJECT_NAME})

# add_executable(mono_euroc
# Examples/Monocular/mono_euroc.cc)
# target_link_libraries(mono_euroc ${PROJECT_NAME})


# ---------------
# Installation instructions.
# ---------------
install(TARGETS ${PROJECT_NAME}
    DESTINATION  lib
    COMPONENT library)
install(FILES
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${G2O_LIBRARIES}
    DESTINATION lib
    COMPONENT library)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION include/ORB_SLAM2
    COMPONENT library
    FILES_MATCHING PATTERN "*.h"
    PATTERN "Thirdparty" EXCLUDE)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/Thirdparty/
    DESTINATION include/ORB_SLAM2/Thirdparty
    COMPONENT library
FILES_MATCHING REGEX ".*\\.h(pp)?")
# ----------------------------------------------------------------------------
# Python Bindings (Optional)
# ----------------------------------------------------------------------------
if(BUILD_PYTHON_BINDINGS)
    message(STATUS "Configuring Python bindings...")    
    add_subdirectory(python) 
else()
    message(STATUS "Skipping Python bindings configuration.")
endif()
