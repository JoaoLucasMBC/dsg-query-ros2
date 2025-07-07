cmake_minimum_required(VERSION 3.8)
project(dsg_query)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hydra_msgs REQUIRED)
find_package(spark_dsg REQUIRED)

add_executable(dsg_query_node src/dsg_query_node.cpp)
add_executable(dsg_path_planner_node src/dsg_path_planner_node.cpp)

ament_target_dependencies(dsg_path_planner_node
  rclcpp
  hydra_msgs
  spark_dsg
)
ament_target_dependencies(dsg_query_node
  rclcpp
  hydra_msgs
  spark_dsg
)


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

install(TARGETS 
  dsg_query_node
  dsg_path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
