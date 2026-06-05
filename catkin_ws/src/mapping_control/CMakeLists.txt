cmake_minimum_required(VERSION 3.0.2)
project(mapping_control)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_msgs nav_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mapping_policy
  src/policy.cpp
)

add_executable(mapping_control_node
  src/mapping_control_node.cpp
)
target_link_libraries(mapping_control_node
  mapping_policy
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(mapping_policy_test test/mapping_policy_test.cpp)
  if(TARGET mapping_policy_test)
    target_link_libraries(mapping_policy_test mapping_policy ${catkin_LIBRARIES})
  endif()
  roslaunch_add_file_check(launch/mapping_control.launch)
  set(_mapping_control_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_mapping_control.launch)
  if(TARGET ${_mapping_control_launch_check_target})
    add_dependencies(${_mapping_control_launch_check_target} mapping_control_node)
  endif()
endif()

install(TARGETS mapping_policy mapping_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
