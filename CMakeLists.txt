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

# ## add eigen functionality
# find_package(Eigen REQUIRED)
# include_directories(${EIGEN_INCLUDE_DIRS})
# add_definitions(${EIGEN_DEFINITIONS})

# rosbuild_add_executable(robot_simulator src/robot_simulator.cpp)
# rosbuild_add_executable(rand_test src/rand_test.cpp)
# rosbuild_add_executable(mass_marker src/mass_marker.cpp)
# rosbuild_add_executable(planar_coordinator src/planar_coordinator.cpp)
# rosbuild_add_boost_directories()
# rosbuild_link_boost(planar_coordinator signals)
# # add logging functionality
# find_library(LOG4CXX_LIBRARY log4cxx)
# target_link_libraries(planar_coordinator ${LOG4CXX_LIBRARY})
