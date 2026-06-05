cmake_minimum_required(VERSION 3.0.2)
project(section_manager)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

add_service_files(
  FILES
  ExportSections.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_msgs message_runtime roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(section_extraction
  src/section_extraction.cpp
)

add_library(section_control_fields
  src/control_field_parser.cpp
)

add_executable(section_manager_node
  src/section_manager_node.cpp
)
add_dependencies(section_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(section_manager_node
  section_control_fields
  section_extraction
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  catkin_add_gtest(section_extraction_test test/section_extraction_test.cpp)
  if(TARGET section_extraction_test)
    target_link_libraries(section_extraction_test section_extraction ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(section_control_field_parser_test test/control_field_parser_test.cpp)
  if(TARGET section_control_field_parser_test)
    target_link_libraries(section_control_field_parser_test section_control_fields ${catkin_LIBRARIES})
  endif()
  roslaunch_add_file_check(launch/section_manager.launch)
  set(_section_manager_launch_check_target run_tests_${PROJECT_NAME}_roslaunch-check_launch_section_manager.launch)
  if(TARGET ${_section_manager_launch_check_target})
    add_dependencies(${_section_manager_launch_check_target} section_manager_node)
  endif()
endif()

install(TARGETS section_control_fields section_extraction section_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
