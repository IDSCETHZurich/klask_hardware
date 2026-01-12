cmake_minimum_required(VERSION 3.8)
project(klask_motor_commander_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(odrive_can REQUIRED)
find_package(klask_interfaces REQUIRED)

add_executable(klask_motor_commander
  src/klask_motor_commander.cpp
  src/player.cpp
  src/odrive_controller.cpp
  src/open_loop_controller.cpp
)

ament_target_dependencies(klask_motor_commander rclcpp rclcpp_action std_msgs geometry_msgs odrive_can klask_interfaces)

target_include_directories(klask_motor_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(klask_motor_commander PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS klask_motor_commander
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Add custom clang-format check test
  find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)
  if(CLANG_FORMAT_EXECUTABLE)
    file(GLOB_RECURSE ALL_SOURCE_FILES
      ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
      ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
      ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
      ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp
    )

    add_test(
      NAME clang_format_check
      COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/check_clang_format.sh
        ${CMAKE_SOURCE_DIR}
        ${ALL_SOURCE_FILES}
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    set_tests_properties(clang_format_check PROPERTIES
      LABELS "linter;clang_format"
    )
  else()
    message(WARNING "clang-format not found, skipping format check")
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
