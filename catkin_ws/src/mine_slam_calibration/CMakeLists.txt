cmake_minimum_required(VERSION 3.0.2)
project(mine_slam_calibration)

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  tf2
  tf2_ros
)

add_service_files(
  FILES
  AuditTf.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp std_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(mine_slam_calibration_core
  src/extrinsics.cpp
)
target_link_libraries(mine_slam_calibration_core
  yaml-cpp
)

add_executable(publish_extrinsics_node
  src/publish_extrinsics_node.cpp
)
add_dependencies(publish_extrinsics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_extrinsics_node
  mine_slam_calibration_core
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(audit_tf_node
  src/audit_tf_node.cpp
)
add_dependencies(audit_tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(audit_tf_node
  mine_slam_calibration_core
  ${catkin_LIBRARIES}
  yaml-cpp
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(extrinsics_test test/extrinsics_test.cpp)
  if(TARGET extrinsics_test)
    target_link_libraries(extrinsics_test mine_slam_calibration_core ${catkin_LIBRARIES} yaml-cpp)
  endif()
  roslaunch_add_file_check(launch/publish_extrinsics.launch)
  roslaunch_add_file_check(launch/audit_tf.launch)
  set(_audit_tf_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_audit_tf.launch)
  if(TARGET ${_audit_tf_launch_check_target})
    add_dependencies(${_audit_tf_launch_check_target} audit_tf_node)
  endif()
  set(_publish_extrinsics_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_publish_extrinsics.launch)
  if(TARGET ${_publish_extrinsics_launch_check_target})
    add_dependencies(${_publish_extrinsics_launch_check_target} publish_extrinsics_node)
  endif()
endif()

install(TARGETS mine_slam_calibration_core publish_extrinsics_node audit_tf_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/audit_extrinsics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
