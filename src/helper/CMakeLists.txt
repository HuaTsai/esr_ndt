cmake_minimum_required(VERSION 2.8.3)
project(helper)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    tf2_ros
    pcl_ros
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "*.cc")

foreach(FILE_PATH ${ALL_SRCS})
  get_filename_component(FILENAME ${FILE_PATH} NAME_WE)
  add_executable(${FILENAME} ${FILE_PATH})
  target_link_libraries(${FILENAME} ${catkin_LIBRARIES})
endforeach(FILE_PATH)
