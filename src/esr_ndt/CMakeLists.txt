cmake_minimum_required(VERSION 2.8.3)
project(esr_ndt)

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
# file(GLOB_RECURSE ALL_TESTS "*_test.cc")
file(GLOB_RECURSE ALL_MAINS "*_main.cc")
list(REMOVE_ITEM ALL_SRCS ${ALL_MAINS})
# list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})

add_library(${PROJECT_NAME} ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

foreach(FILE_PATH ${ALL_MAINS})
  get_filename_component(FILENAME ${FILE_PATH} NAME_WE)
  add_executable(${FILENAME} ${FILE_PATH})
  target_link_libraries(${FILENAME} ${catkin_LIBRARIES} ${PROJECT_NAME})
endforeach(FILE_PATH)

# if (CATKIN_ENABLE_TESTING)
#   foreach(FILE_PATH ${ALL_TESTS})
#     get_filename_component(FILENAME ${FILE_PATH} NAME_WE)
#     catkin_add_gtest(${FILENAME} ${FILE_PATH})
#     target_link_libraries(${FILENAME} ${catkin_LIBRARIES} ${PROJECT_NAME})
#   endforeach(FILE_PATH)
# endif()
