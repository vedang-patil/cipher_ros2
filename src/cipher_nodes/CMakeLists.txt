cmake_minimum_required(VERSION 3.8)
project(cipher_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cipher_interfaces REQUIRED)

add_executable(game_setter src/game_setter.cpp)
ament_target_dependencies(game_setter rclcpp std_msgs cipher_interfaces)

add_executable(main_node src/main_node.cpp)
ament_target_dependencies(main_node rclcpp std_msgs cipher_interfaces)

install(TARGETS
  game_setter
  main_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
