cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## add eigen functionality
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})


## define a function for automatically linking log4cxx:
find_library(LOG4CXX_LIBRARY log4cxx)
function(add_logging_executable name sources)
    rosbuild_add_executable(${name} ${sources})
    target_link_libraries(${name} ${LOG4CXX_LIBRARY})
endfunction(add_logging_executable)

rosbuild_add_executable(receding_coordinator src/receding_coordinator.cpp)
rosbuild_link_boost(receding_coordinator signals)

## add video_recorder executable
find_package(OpenCV REQUIRED)
rosbuild_add_executable(video_recorder src/video_recorder.cpp)
rosbuild_link_boost(video_recorder signals)
target_link_libraries(video_recorder ${OpenCV_LIBS})

# add executable for the basic_controls.cpp
rosbuild_add_executable(basic_controls src/basic_controls.cpp)