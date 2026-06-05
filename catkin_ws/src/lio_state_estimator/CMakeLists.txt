cmake_minimum_required(VERSION 3.0.2)
project(lio_state_estimator)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  nav_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_msgs nav_msgs roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lio_state_estimator_core
  src/imu_window.cpp
  src/preintegration.cpp
)

add_executable(state_estimator_node
  src/state_estimator_node.cpp
)
target_link_libraries(state_estimator_node
  lio_state_estimator_core
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(imu_window_test test/imu_window_test.cpp)
  if(TARGET imu_window_test)
    target_link_libraries(imu_window_test lio_state_estimator_core ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(preintegration_test test/preintegration_test.cpp)
  if(TARGET preintegration_test)
    target_link_libraries(preintegration_test lio_state_estimator_core ${catkin_LIBRARIES})
  endif()
  roslaunch_add_file_check(launch/state_estimator.launch)
  set(_state_estimator_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_state_estimator.launch)
  if(TARGET ${_state_estimator_launch_check_target})
    add_dependencies(${_state_estimator_launch_check_target} state_estimator_node)
  endif()
endif()

install(TARGETS lio_state_estimator_core state_estimator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
