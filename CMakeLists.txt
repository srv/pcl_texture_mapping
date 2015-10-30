cmake_minimum_required(VERSION 2.8.3)
project(pcl_texture_mapping)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs image_geometry image_transport pcl_ros roscpp sensor_msgs tf
  DEPENDS Boost PCL OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcl_texture_mapping_node src/node.cpp src/texture_mapper.cpp)
target_link_libraries(pcl_texture_mapping_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
