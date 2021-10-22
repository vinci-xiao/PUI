cmake_minimum_required(VERSION 3.5)
project(pui_uwb)

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
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pui_msgs REQUIRED)   # for customed msg (MultiRange)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(trilateration_mse src/trilateration_mse.cpp)
ament_target_dependencies(trilateration_mse rclcpp std_msgs geometry_msgs pui_msgs Eigen3 tf2_msgs tf2 tf2_ros)

add_executable(trilateration_mse_3d src/trilateration_mse_3d.cpp)
ament_target_dependencies(trilateration_mse_3d rclcpp std_msgs geometry_msgs pui_msgs Eigen3 tf2_msgs tf2 tf2_ros)

add_executable(following_node src/following_node.cpp)
ament_target_dependencies(following_node rclcpp std_msgs geometry_msgs pui_msgs)

install(TARGETS
  trilateration_mse
  following_node
  trilateration_mse_3d
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

ament_package()
