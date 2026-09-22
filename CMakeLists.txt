cmake_minimum_required(VERSION 3.8)
project(athena_action_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(material_handler_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(standard_msgs REQUIRED)

add_executable(action_bridge_node src/action_bridge_node.cpp)

target_include_directories(action_bridge_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(action_bridge_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${material_handler_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${standard_msgs_TARGETS}
)

install(TARGETS action_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
