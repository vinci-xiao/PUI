cmake_minimum_required(VERSION 3.5)
project(pui_node)

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

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)
add_library(${PROJECT_NAME}_lib

  "src/diff_drive_controller.cpp"
  "src/dynamixel_controller.cpp"
  "src/odometry.cpp"
  "src/pui.cpp"

  # "src/sensors/battery_state.cpp"
  # "src/sensors/imu.cpp"
  # "src/sensors/joint_state.cpp"
  # "src/sensors/sensor_state.cpp"
)

set(DEPENDENCIES
  "dynamixel_sdk"
  "geometry_msgs"
  "message_filters"
  "nav_msgs"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(EXECUTABLE_NAME "pui_ros")
add_executable(${EXECUTABLE_NAME} src/node_main.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

add_executable(dynamixel_controller src/dynamixel_controller.cpp)
target_link_libraries(dynamixel_controller ${PROJECT_NAME}_lib)
ament_target_dependencies(dynamixel_controller ${DEPENDENCIES})
################################################################################
# Install
################################################################################
install(DIRECTORY param include
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  ${EXECUTABLE_NAME}
  dynamixel_controller
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()
