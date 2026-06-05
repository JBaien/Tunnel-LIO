cmake_minimum_required(VERSION 3.0.2)
project(mine_slam_bringup)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_service_files(
  FILES
  DumpEvent.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(event_dump
  src/event_dump.cpp
)

add_executable(diag_dump_event_node
  src/diag_dump_event_node.cpp
)
add_dependencies(diag_dump_event_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(diag_dump_event_node
  event_dump
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(event_dump_test test/event_dump_test.cpp)
  if(TARGET event_dump_test)
    target_link_libraries(event_dump_test event_dump ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(record_session_script_test test/record_session_script_test.cpp)
  if(TARGET record_session_script_test)
    target_compile_definitions(record_session_script_test PRIVATE
      RECORD_SESSION_SCRIPT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/scripts/record_session.sh")
  endif()
  catkin_add_gtest(runtime_ops_script_test test/runtime_ops_script_test.cpp)
  if(TARGET runtime_ops_script_test)
    target_compile_definitions(runtime_ops_script_test PRIVATE
      RUNTIME_OPS_SCRIPT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/scripts/runtime_ops.sh")
  endif()
  catkin_add_gtest(actual_bag_replay_script_test test/actual_bag_replay_script_test.cpp)
  if(TARGET actual_bag_replay_script_test)
    target_compile_definitions(actual_bag_replay_script_test PRIVATE
      ACTUAL_BAG_REPLAY_SCRIPT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/scripts/actual_bag_replay.sh")
  endif()
  catkin_add_gtest(actual_bag_profile_script_test test/actual_bag_profile_script_test.cpp)
  if(TARGET actual_bag_profile_script_test)
    target_compile_definitions(actual_bag_profile_script_test PRIVATE
      ACTUAL_BAG_PROFILE_SCRIPT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/scripts/actual_bag_profile.sh")
  endif()
  catkin_add_gtest(actual_bag_test_suite_script_test test/actual_bag_test_suite_script_test.cpp)
  if(TARGET actual_bag_test_suite_script_test)
    target_compile_definitions(actual_bag_test_suite_script_test PRIVATE
      ACTUAL_BAG_TEST_SUITE_SCRIPT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/scripts/actual_bag_test_suite.sh")
  endif()
  catkin_add_gtest(ros_language_contract_test test/ros_language_contract_test.cpp)
  if(TARGET ros_language_contract_test)
    target_compile_definitions(ros_language_contract_test PRIVATE
      CATKIN_WORKSPACE_SRC_DIR="${CMAKE_SOURCE_DIR}")
  endif()
  catkin_add_gtest(bringup_launch_contract_test test/bringup_launch_contract_test.cpp)
  if(TARGET bringup_launch_contract_test)
    target_compile_definitions(bringup_launch_contract_test PRIVATE
      BRINGUP_LAUNCH_DIR="${CMAKE_CURRENT_SOURCE_DIR}/launch"
      CATKIN_WORKSPACE_SRC_DIR="${CMAKE_SOURCE_DIR}"
      SECTION_MANAGER_LAUNCH_PATH="${CMAKE_SOURCE_DIR}/section_manager/launch/section_manager.launch")
  endif()
  roslaunch_add_file_check(launch/bringup_sensors.launch)
  roslaunch_add_file_check(launch/bringup_fusion_timoo.launch)
  roslaunch_add_file_check(launch/bringup_fusion_tmlidar.launch)
  roslaunch_add_file_check(launch/bringup_replay_tunnel_bag.launch)
  roslaunch_add_file_check(launch/diag_dump_event.launch)
  set(_diag_dump_event_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_diag_dump_event.launch)
  if(TARGET ${_diag_dump_event_launch_check_target})
    add_dependencies(${_diag_dump_event_launch_check_target} diag_dump_event_node)
  endif()
  roslaunch_add_file_check(launch/record_session.launch)
endif()

install(TARGETS event_dump diag_dump_event_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS
  scripts/actual_bag_profile.sh
  scripts/actual_bag_replay.sh
  scripts/actual_bag_test_suite.sh
  scripts/record_session.sh
  scripts/runtime_ops.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
