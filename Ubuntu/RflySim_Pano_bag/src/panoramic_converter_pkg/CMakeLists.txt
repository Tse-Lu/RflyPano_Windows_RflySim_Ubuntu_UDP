cmake_minimum_required(VERSION 3.0.2)
project(panoramic_converter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge image_transport sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pano_to_catadioptric src/pano_to_catadioptric.cpp)
target_link_libraries(pano_to_catadioptric
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(pano_to_4fisheye src/pano_to_4fisheye.cpp)
target_link_libraries(pano_to_4fisheye
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(compressed_to_raw src/compressed_to_raw.cpp)
target_link_libraries(compressed_to_raw
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS pano_to_catadioptric pano_to_4fisheye compressed_to_raw
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
