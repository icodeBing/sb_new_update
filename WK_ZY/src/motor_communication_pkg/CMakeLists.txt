cmake_minimum_required(VERSION 3.10)
project(motor_communication_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motor_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Necro REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  include
  ../../../
)

set(SRC_LISTS
  src/main.cpp
  src/motor_comm/motorComm.cpp
  src/motor_comm/IPCComm.cpp
  src/motor_comm/logRecord.cpp
  src/global/globalParams.cpp
)

add_executable(motor_node ${SRC_LISTS})

ament_target_dependencies(motor_node
  rclcpp
  motor_interface
  sensor_msgs
)

target_link_libraries(motor_node Necro::qiuniu Threads::Threads)

# Necro特定引导
necro_target_bootstrap(motor_node) 

install(TARGETS
  motor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
