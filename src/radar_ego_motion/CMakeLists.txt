cmake_minimum_required(VERSION 2.8.3)
project(radar_ego_motion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  esr_msgs
  pcl_ros
)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(esr_ego_motion src/esr_ego_motion.cpp)
target_link_libraries(esr_ego_motion ${catkin_LIBRARIES})

install(
  TARGETS
    esr_ego_motion
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
