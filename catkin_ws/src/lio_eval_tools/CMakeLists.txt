cmake_minimum_required(VERSION 3.0.2)
project(lio_eval_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lio_eval_core
  src/evidence_manifest.cpp
  src/replay_metric_accumulator.cpp
  src/validation_metrics.cpp
)
target_link_libraries(lio_eval_core
  ${catkin_LIBRARIES}
)

add_executable(validation_report_node
  src/validation_report_node.cpp
)
target_link_libraries(validation_report_node
  lio_eval_core
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(validation_metrics_test test/validation_metrics_test.cpp)
  if(TARGET validation_metrics_test)
    target_link_libraries(validation_metrics_test lio_eval_core ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(replay_metric_accumulator_test test/replay_metric_accumulator_test.cpp)
  if(TARGET replay_metric_accumulator_test)
    target_link_libraries(replay_metric_accumulator_test lio_eval_core ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(evidence_manifest_test test/evidence_manifest_test.cpp)
  if(TARGET evidence_manifest_test)
    target_compile_definitions(evidence_manifest_test PRIVATE
      LIO_EVAL_TOOLS_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
    target_link_libraries(evidence_manifest_test lio_eval_core ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(validation_launch_contract_test test/validation_launch_contract_test.cpp)
  if(TARGET validation_launch_contract_test)
    target_compile_definitions(validation_launch_contract_test PRIVATE
      VALIDATION_REPORT_LAUNCH_PATH="${CMAKE_CURRENT_SOURCE_DIR}/launch/validation_report.launch")
  endif()
  roslaunch_add_file_check(launch/validation_report.launch)
  set(_validation_report_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_validation_report.launch)
  if(TARGET ${_validation_report_launch_check_target})
    add_dependencies(${_validation_report_launch_check_target} validation_report_node)
  endif()
endif()

install(TARGETS lio_eval_core validation_report_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
