cmake_minimum_required(VERSION 3.8)
project(cpp_srvcli)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(server src/add_three_ints_server.cpp)
ament_target_dependencies(server rclcpp custom_interfaces)

add_executable(client src/add_three_ints_client.cpp)
ament_target_dependencies(client rclcpp custom_interfaces)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
