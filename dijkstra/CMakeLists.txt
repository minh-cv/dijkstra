cmake_minimum_required(VERSION 3.8)
project(dijkstra)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
	
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NodesPair.msg"
  "msg/NodeInfo.msg"
  "action/Action.action"
)

include_directories(include)

add_library(map STATIC src/map.cpp)

add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp rclcpp_action)
target_link_libraries(server map)

add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp rclcpp_action)
target_link_libraries(client map)

install(
  TARGETS server client map
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(server "${cpp_typesupport_target}")
target_link_libraries(client "${cpp_typesupport_target}")
target_link_libraries(map "${cpp_typesupport_target}")

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
