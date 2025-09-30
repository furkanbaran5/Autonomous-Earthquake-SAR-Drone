cmake_minimum_required(VERSION 3.0.2)
project(firefly_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  detection_msgs
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  octomap_msgs
  octomap_ros
  octomap
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/FrontierDetection.cfg
)

find_package(octomap REQUIRED)
catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_control src/keyboard_control.cpp)
target_link_libraries(keyboard_control ${catkin_LIBRARIES})
add_dependencies(keyboard_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(frontier_detection src/frontier_detection.cpp)
target_link_libraries(frontier_detection ${catkin_LIBRARIES})
add_dependencies(frontier_detection ${catkin_EXPORTED_TARGETS})

add_executable(up src/up.cpp)
target_link_libraries(up ${catkin_LIBRARIES})
add_dependencies(up ${catkin_EXPORTED_TARGETS})

add_executable(follow_path src/follow_path.cpp)
target_link_libraries(follow_path ${catkin_LIBRARIES})
add_dependencies(follow_path ${catkin_EXPORTED_TARGETS})

add_executable(costmap_build src/costmap_build.cpp)
target_link_libraries(costmap_build ${catkin_LIBRARIES})
add_dependencies(costmap_build ${catkin_EXPORTED_TARGETS})

add_executable(create_path src/create_path.cpp)
target_link_libraries(create_path ${catkin_LIBRARIES})
add_dependencies(create_path ${catkin_EXPORTED_TARGETS})

add_executable(firefly_trajectory src/firefly_trajectory.cpp)
target_link_libraries(firefly_trajectory ${catkin_LIBRARIES})
add_dependencies(firefly_trajectory ${catkin_EXPORTED_TARGETS})

add_executable(turn_360 src/turn_360.cpp)
target_link_libraries(turn_360 ${catkin_LIBRARIES})
add_dependencies(turn_360 ${catkin_EXPORTED_TARGETS})

add_executable(move_corner_build src/move_corner_build.cpp)
target_link_libraries(move_corner_build ${catkin_LIBRARIES})
add_dependencies(move_corner_build ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  src/fsm_states.py  
  src/altitude_scanner.py
  src/building_detector.py
  src/smart_building_selector.py
  src/pop_cordinate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
