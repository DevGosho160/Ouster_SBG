cmake_minimum_required(VERSION 3.5)
project(ouster_logger_c)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Open3D REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Open3D_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Open3D_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Executable
add_executable(ouster_logger_c src/ouster_logger_c.cpp)

ament_target_dependencies(ouster_logger_c
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(ouster_logger_c
  ${PCL_LIBRARIES}
  Open3D::Open3D
)

install(TARGETS
  ouster_logger_c
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
