cmake_minimum_required(VERSION 3.5)
project(pui_markers)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(tag_path src/tag_path.cpp)
ament_target_dependencies(tag_path rclcpp geometry_msgs std_msgs nav_msgs visualization_msgs)

add_executable(tag_path_3d src/tag_path_3d.cpp)
ament_target_dependencies(tag_path_3d rclcpp geometry_msgs std_msgs nav_msgs)

add_executable(uwb_marker src/uwb_marker.cpp)
ament_target_dependencies(uwb_marker rclcpp visualization_msgs geometry_msgs)

install(TARGETS
  uwb_marker
  tag_path
  tag_path_3d
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
