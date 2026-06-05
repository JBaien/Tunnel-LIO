cmake_minimum_required(VERSION 3.0.2)
project(machine_state_manager)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

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
  ${LIBMODBUS_INCLUDE_DIRS}
)

add_library(machine_state_logic
  src/state_logic.cpp
)

add_executable(machine_state_node
  src/machine_state_node.cpp
)
target_link_libraries(machine_state_node
  machine_state_logic
  ${catkin_LIBRARIES}
)

add_executable(plc_modbus_node
  src/plc_modbus_node.cpp
)
target_link_libraries(plc_modbus_node
  ${catkin_LIBRARIES}
  ${LIBMODBUS_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(machine_state_logic_test test/machine_state_logic_test.cpp)
  if(TARGET machine_state_logic_test)
    target_link_libraries(machine_state_logic_test machine_state_logic ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(plc_modbus_parser_test test/plc_modbus_parser_test.cpp)
  if(TARGET plc_modbus_parser_test)
    target_link_libraries(plc_modbus_parser_test machine_state_logic ${catkin_LIBRARIES})
  endif()
  roslaunch_add_file_check(launch/machine_state.launch)
  set(_machine_state_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_machine_state.launch)
  if(TARGET ${_machine_state_launch_check_target})
    add_dependencies(${_machine_state_launch_check_target} machine_state_node)
    add_dependencies(${_machine_state_launch_check_target} plc_modbus_node)
  endif()
endif()

install(TARGETS machine_state_logic machine_state_node plc_modbus_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
