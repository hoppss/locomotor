cmake_minimum_required(VERSION 3.5)
project(locomotor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -pthread")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  angles
  tf2
  tf2_sensor_msgs
  tf2_ros
  nav2_core
  nav2_util
  nav2_msgs
  nav_msgs
  sensor_msgs
  nav2_costmap_2d
  rclcpp_lifecycle
)

add_executable(locomotor src/Locomotor.cpp)
ament_target_dependencies(locomotor ${dependencies})

add_executable(timer src/timer.cpp)
ament_target_dependencies(timer ${dependencies})

install(TARGETS
        locomotor timer
        DESTINATION lib/${PROJECT_NAME})

ament_package()
