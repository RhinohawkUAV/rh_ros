cmake_minimum_required(VERSION 2.8.3)
project(rh_vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(
    OpenCV
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rh_vision
#  CATKIN_DEPENDS opencv2 roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(canny_edges src/canny_edges.cpp)
add_executable(threshold src/threshold.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(canny_edges
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
target_link_libraries(threshold
  ${catkin_LIBRARIES}
  ${openCV_LIBRARIES}
)
