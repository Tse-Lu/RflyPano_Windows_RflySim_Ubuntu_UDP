cmake_minimum_required(VERSION 2.8.3)
project(rflysim_ros_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  message_generation
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  Obj.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(img_node 
  src/image_receive.cpp
)

add_executable(imu_node
  src/imu_receive.cpp
)

target_link_libraries(img_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

target_link_libraries(imu_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
