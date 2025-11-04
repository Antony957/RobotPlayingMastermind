cmake_minimum_required(VERSION 3.8)
project(mastermind)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GuessCheck.msg"
  "msg/Status.msg"
  "msg/Code.msg"
)

ament_python_install_package(${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()