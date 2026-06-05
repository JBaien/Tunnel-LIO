cmake_minimum_required(VERSION 3.0.2)
project(lio_local_odometry)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lio_local_registration
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs
  DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(lio_local_registration
  src/local_registration.cpp
)
target_link_libraries(lio_local_registration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(local_icp_odometry_node
  src/local_icp_odometry_node.cpp
)
target_link_libraries(local_icp_odometry_node
  lio_local_registration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(local_odometry_math_test test/local_odometry_math_test.cpp)
  if(TARGET local_odometry_math_test)
    target_link_libraries(local_odometry_math_test ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(local_registration_test test/local_registration_test.cpp)
  if(TARGET local_registration_test)
    target_link_libraries(local_registration_test lio_local_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  roslaunch_add_file_check(launch/local_icp_odometry.launch)
  set(_local_icp_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_local_icp_odometry.launch)
  if(TARGET ${_local_icp_launch_check_target})
    add_dependencies(${_local_icp_launch_check_target} local_icp_odometry_node)
  endif()
endif()

install(TARGETS lio_local_registration local_icp_odometry_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
