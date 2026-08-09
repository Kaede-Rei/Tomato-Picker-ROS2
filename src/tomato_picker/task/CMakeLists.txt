cmake_minimum_required(VERSION 3.8)
project(tomato_picker_task)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tomato_picker_interfaces REQUIRED)

add_executable(pick_task_node src/pick_task_node.cpp)
ament_target_dependencies(pick_task_node geometry_msgs rclcpp rclcpp_action tomato_picker_interfaces)

install(TARGETS pick_task_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
