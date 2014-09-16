cmake_minimum_required(VERSION 2.8.3)
project(receding_planar_sys)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  rospy roscpp std_msgs tf tf_conversions
  angles nav_msgs visualization_msgs puppeteer_msgs cmake_modules cv_bridge
  image_transport camera_calibration_parsers std_srvs interactive_markers
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(OpenCV REQUIRED )
find_package(Boost REQUIRED COMPONENTS system signals)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES test
  CATKIN_DEPENDS rospy roscpp std_msgs tf tf_conversions angles nav_msgs visualization_msgs puppeteer_msgs cv_bridge image_transport camera_calibration_parsers std_srvs interactive_markers
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
 include ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(receding_coordinator src/receding_coordinator.cpp)
add_executable(video_recorder src/video_recorder.cpp)
add_executable(interactive_marker_ref_gen src/interactive_marker_ref_gen.cpp)

find_library(LOG4CXX_LIBRARY log4cxx)
## Specify libraries to link a library or executable target against
 target_link_libraries(
  receding_coordinator ${catkin_LIBRARIES} ${Boost_LIBRARIES}
 )
 target_link_libraries(
  video_recorder ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
 )
 target_link_libraries(
  interactive_marker_ref_gen ${catkin_LIBRARIES}
 )

