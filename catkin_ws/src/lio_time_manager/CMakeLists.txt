cmake_minimum_required(VERSION 3.0.2)
project(lio_time_manager)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_msgs roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lio_time_status
  src/time_status.cpp
)

add_executable(time_status_node
  src/time_status_node.cpp
)
target_link_libraries(time_status_node
  lio_time_status
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(time_status_test test/time_status_test.cpp)
  if(TARGET time_status_test)
    target_link_libraries(time_status_test lio_time_status ${catkin_LIBRARIES})
  endif()
  roslaunch_add_file_check(launch/time_status.launch)
  set(_time_status_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_time_status.launch)
  if(TARGET ${_time_status_launch_check_target})
    add_dependencies(${_time_status_launch_check_target} time_status_node)
  endif()
endif()

install(TARGETS lio_time_status time_status_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
