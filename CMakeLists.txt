cmake_minimum_required(VERSION 3.4)
project(pose_verification)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  visualization_msgs
  geometry_msgs
  sensor_msgs
  concert_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_verification
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    visualization_msgs
    geometry_msgs
    sensor_msgs
    concert_msgs
)

add_executable(pose_verification
  src/pose_verification.cc
)

add_dependencies(pose_verification
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pose_verification PUBLIC
  ${catkin_LIBRARIES}
)

target_include_directories(pose_verification SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(pose_verification PUBLIC
  include
)
