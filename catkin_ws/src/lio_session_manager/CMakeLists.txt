cmake_minimum_required(VERSION 3.0.2)
project(lio_session_manager)

find_package(catkin REQUIRED COMPONENTS
  lio_local_odometry
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(PCL REQUIRED COMPONENTS common kdtree)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS lio_local_odometry pcl_conversions roscpp sensor_msgs std_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(lio_session_store
  src/session_store.cpp
)
target_link_libraries(lio_session_store
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(session_manager_node
  src/session_manager_node.cpp
)
target_link_libraries(session_manager_node
  lio_session_store
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(session_store_test test/session_store_test.cpp)
  if(TARGET session_store_test)
    target_link_libraries(session_store_test lio_session_store ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  roslaunch_add_file_check(launch/session_manager.launch)
  set(_session_manager_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_session_manager.launch)
  if(TARGET ${_session_manager_launch_check_target})
    add_dependencies(${_session_manager_launch_check_target} session_manager_node)
  endif()
endif()

install(TARGETS lio_session_store session_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
