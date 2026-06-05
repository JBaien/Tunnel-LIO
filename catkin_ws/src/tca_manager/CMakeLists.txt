cmake_minimum_required(VERSION 3.0.2)
project(tca_manager)

find_package(Boost REQUIRED COMPONENTS filesystem system)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_msgs roscpp sensor_msgs std_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(tca_manager_core
  src/tca_detection.cpp
  src/tca_point_cloud2.cpp
)
target_link_libraries(tca_manager_core
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(tca_manager_node
  src/tca_manager_node.cpp
)
target_link_libraries(tca_manager_node
  tca_manager_core
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(tca_detection_test test/tca_detection_test.cpp)
  if(TARGET tca_detection_test)
    target_link_libraries(tca_detection_test tca_manager_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  endif()
  roslaunch_add_file_check(launch/tca_manager.launch)
  set(_tca_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_tca_manager.launch)
  if(TARGET ${_tca_launch_check_target})
    add_dependencies(${_tca_launch_check_target} tca_manager_node)
  endif()
endif()

install(TARGETS tca_manager_core tca_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
