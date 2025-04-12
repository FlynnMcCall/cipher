cmake_minimum_required(VERSION 3.8)
project(caesar_e)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cipher_interfaces REQUIRED)

add_executable(fabian_client src/client_pub.cpp)
ament_target_dependencies(fabian_client rclcpp std_msgs cipher_interfaces)

add_executable(cipher_node src/cipher_node.cpp)
ament_target_dependencies(cipher_node rclcpp std_msgs cipher_interfaces)


install(TARGETS
  fabian_client
  cipher_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
