cmake_minimum_required(VERSION 3.8)
project(tomato_picker_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tomato_picker_interfaces REQUIRED)

add_executable(cloud_preprocessor src/cloud_preprocessor.cpp)
target_include_directories(cloud_preprocessor PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_preprocessor ${PCL_LIBRARIES})
ament_target_dependencies(cloud_preprocessor
  cv_bridge
  message_filters
  pcl_conversions
  rclcpp
  rmw
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tomato_picker_interfaces
)

install(TARGETS cloud_preprocessor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
