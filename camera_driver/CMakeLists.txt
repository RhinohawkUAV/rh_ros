cmake_minimum_required(VERSION 2.8.3)
project(camera_driver)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cv_bridge
  image_transport
  OpenCV
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libgphoto2 REQUIRED)
pkg_check_modules(libraw REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${libgphoto2_INCLUDE_DIRS})

add_executable(camera_image src/camera_image_node.cpp)
add_executable(camera_driver src/camera_driver_node.cpp)
add_executable(camera_video src/camera_video_node.cpp)
#add_executable(camera_v4l2 src/camera_v4l2_node.cpp)  
add_executable(nikon_d5100 src/nikon_d5100.cpp)

target_link_libraries(camera_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(camera_driver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(camera_video ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#target_link_libraries(camera_v4l2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(nikon_d5100 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gphoto2)


