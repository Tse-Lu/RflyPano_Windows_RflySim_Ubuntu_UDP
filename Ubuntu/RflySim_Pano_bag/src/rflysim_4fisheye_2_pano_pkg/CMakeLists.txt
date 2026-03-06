cmake_minimum_required(VERSION 2.8.3)
project(rflysim_4fisheye_2_pano_pkg)

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
find_package(Eigen3 REQUIRED)


catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(pano_node src/four_fisheye_2_pano.cpp)

target_link_libraries(pano_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
