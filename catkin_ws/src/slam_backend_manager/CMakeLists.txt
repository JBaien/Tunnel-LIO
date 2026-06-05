cmake_minimum_required(VERSION 3.0.2)
project(slam_backend_manager)

find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Ceres REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_msgs nav_msgs roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_library(slam_backend_core
  src/backend_candidates.cpp
  src/backend_point_cloud_reader.cpp
  src/control_field_parser.cpp
  src/pose_graph_optimizer.cpp
  src/stable_map_store.cpp
)
target_link_libraries(slam_backend_core
  ${Boost_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(slam_backend_node
  src/slam_backend_node.cpp
)
target_link_libraries(slam_backend_node
  slam_backend_core
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(backend_candidates_test test/backend_candidates_test.cpp)
  if(TARGET backend_candidates_test)
    target_link_libraries(backend_candidates_test slam_backend_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  endif()
  catkin_add_gtest(backend_point_cloud_reader_test test/backend_point_cloud_reader_test.cpp)
  if(TARGET backend_point_cloud_reader_test)
    target_link_libraries(backend_point_cloud_reader_test slam_backend_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  endif()
  catkin_add_gtest(pose_graph_optimizer_test test/pose_graph_optimizer_test.cpp)
  if(TARGET pose_graph_optimizer_test)
    target_link_libraries(pose_graph_optimizer_test slam_backend_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  endif()
  catkin_add_gtest(stable_map_store_test test/stable_map_store_test.cpp)
  if(TARGET stable_map_store_test)
    target_link_libraries(stable_map_store_test slam_backend_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  endif()
  catkin_add_gtest(slam_backend_control_field_parser_test test/control_field_parser_test.cpp)
  if(TARGET slam_backend_control_field_parser_test)
    target_link_libraries(slam_backend_control_field_parser_test slam_backend_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  endif()
  roslaunch_add_file_check(launch/slam_backend.launch)
  set(_slam_backend_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_slam_backend.launch)
  if(TARGET ${_slam_backend_launch_check_target})
    add_dependencies(${_slam_backend_launch_check_target} slam_backend_node)
  endif()
endif()

install(TARGETS slam_backend_core slam_backend_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
