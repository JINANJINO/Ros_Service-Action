cmake_minimum_required(VERSION 3.8)
project(msg_interface_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find and load build settings from external packages
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/TwoTextMessage.msg"
  "msg/TwoNumSum.msg"
)

set(srv_files
  "srv/PlusService.srv"
  "srv/TwoStringService.srv"
)

set(action_files
  "action/Fibonacci.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
