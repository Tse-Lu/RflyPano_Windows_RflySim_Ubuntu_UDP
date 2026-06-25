cmake_minimum_required(VERSION 3.10)
project(rflysim_4fisheye_2_pano_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)
# This node no longer uses cv_bridge, so it can use one consistent OpenCV
# version on its own. Force the system OpenCV (4.5.4) config so we don't pick
# up the /usr/local 4.13.0 build, which avoids loading two OpenCV versions at
# runtime (the previous SIGSEGV cause).
set(OpenCV_DIR /usr/lib/aarch64-linux-gnu/cmake/opencv4)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Safety: drop any /usr/local OpenCV (4.13.0) a catkin dependency might drag in.
list(FILTER catkin_LIBRARIES EXCLUDE REGEX "/usr/local/.*opencv")
list(FILTER catkin_INCLUDE_DIRS EXCLUDE REGEX "/usr/local/include/opencv")
message(STATUS "pano_node OpenCV version: ${OpenCV_VERSION}")

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pano_node src/four_fisheye_2_pano.cpp)

target_link_libraries(pano_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
