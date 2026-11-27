cmake_minimum_required(VERSION 3.5)
project(launch_testing_ros_ament_cmake)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  include(cmake/add_launch_ros_test.cmake)

  ament_index_has_resource(LAUNCH_TESTING_INSTALL_PREFIX packages launch_testing)
  if(NOT LAUNCH_TESTING_INSTALL_PREFIX)
      message(FATAL_ERROR "launch_testing package not found")
  endif()

  # Test argument passing.  This test won't pass unless you give it an argument
  add_launch_ros_test(
    "${LAUNCH_TESTING_INSTALL_PREFIX}/share/launch_testing/examples/args_launch_test.py"
    ARGS "dut_arg:=--anything"
  )
endif()

ament_export_dependencies(launch_testing_ament_cmake)

ament_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
)
