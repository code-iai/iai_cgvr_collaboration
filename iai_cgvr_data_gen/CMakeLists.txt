cmake_minimum_required(VERSION 2.8.3)
project(iai_cgvr_data_gen)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pr2_description
  roscpp
  visualization_msgs
  rospy
  urdfdom_py
  std_srvs
  roslib
  sensor_msgs
  urdf
  cmake_modules
  kdl_parser
  robot_state_publisher
)

find_package(TinyXML2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iai_cgvr_data_gen
#  CATKIN_DEPENDS pr2_description rospy urdf_parser_py
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS})

add_executable(episode_logger src/episode_logger.cpp)
add_dependencies(episode_logger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(episode_logger ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})
