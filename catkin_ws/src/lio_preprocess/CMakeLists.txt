cmake_minimum_required(VERSION 3.0.2)
project(lio_preprocess)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_msgs roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lio_preprocess_core
  src/filtering.cpp
  src/deskewing.cpp
  src/point_cloud2_reader.cpp
)

add_executable(preprocess_node
  src/preprocess_node.cpp
)
target_link_libraries(preprocess_node
  lio_preprocess_core
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(preprocess_filtering_test test/preprocess_filtering_test.cpp)
  if(TARGET preprocess_filtering_test)
    target_link_libraries(preprocess_filtering_test lio_preprocess_core ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(preprocess_point_cloud2_reader_test test/preprocess_point_cloud2_reader_test.cpp)
  if(TARGET preprocess_point_cloud2_reader_test)
    target_link_libraries(preprocess_point_cloud2_reader_test lio_preprocess_core ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(preprocess_deskewing_test test/preprocess_deskewing_test.cpp)
  if(TARGET preprocess_deskewing_test)
    target_link_libraries(preprocess_deskewing_test lio_preprocess_core ${catkin_LIBRARIES})
  endif()
  roslaunch_add_file_check(launch/preprocess.launch)
  set(_preprocess_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_preprocess.launch)
  if(TARGET ${_preprocess_launch_check_target})
    add_dependencies(${_preprocess_launch_check_target} preprocess_node)
  endif()
endif()

install(TARGETS lio_preprocess_core preprocess_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
