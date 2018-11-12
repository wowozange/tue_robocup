cmake_minimum_required(VERSION 2.8.3)
project(challenge_restaurant)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  depth_image_proc
  image_geometry
  image_transport
  pcl_conversion
  pcl_ros
  roscpp
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  GetNormalScore.srv
)

catkin_python_setup()

generate_messages()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(get_normal_score_node src/get_normal_score_node.cpp)
target_link_libraries(get_normal_score_node ${catkin_LIBRARIES})
add_dependencies(get_normal_score_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
