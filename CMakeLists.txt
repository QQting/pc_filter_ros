cmake_minimum_required(VERSION 2.8.3)
project(pc_filter_ros)

set (CMAKE_CXX_STANDARD 11)

set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED sensor_msgs pcl_ros roscpp dynamic_reconfigure)
find_package(PCL)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/PcFilter.cfg
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs pcl_ros dynamic_reconfigure
)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pc_filter src/pc_filter.cpp)
add_dependencies(pc_filter ${PROJECT_NAME}_gencfg)
target_link_libraries(pc_filter ${PCL_LIBS} ${catkin_LIBRARIES})

