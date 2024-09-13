cmake_minimum_required(VERSION 3.5)
project(limo_control)

# C++ settings:
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(transformation_lib src/transformations.cpp)
add_executable(limo_control src/limo_control.cpp)

include_directories(${CMAKE_SOURCE_DIR}/include)

target_link_libraries(limo_control transformation_lib)

set (SHARED_DEPENDENCIES
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
)

ament_target_dependencies(limo_control ${SHARED_DEPENDENCIES})
ament_target_dependencies(transformation_lib ${SHARED_DEPENDENCIES})

install(
    TARGETS
    limo_control
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()