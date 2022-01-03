cmake_minimum_required(VERSION 2.8.3)
project(scan_to_point_cloud)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  COMPONENTS
    laser_geometry
    nodelet
    roscpp
    sensor_msgs
    tf2_ros
)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    laser_geometry
    nodelet
    roscpp
    sensor_msgs
    tf2_ros
  LIBRARIES
    ${PROJECT_NAME}_plugins
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_plugins
  src/scan_to_point_cloud/scan_to_point_cloud.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}_plugins
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  FILES ${PROJECT_NAME}_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
