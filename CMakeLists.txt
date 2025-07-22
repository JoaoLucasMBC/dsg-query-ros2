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
find_package(pose_graph_tools_msgs REQUIRED)
find_package(kimera_pgmo_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io features)
find_package(std_srvs REQUIRED)
find_package(assimp REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(pcl_mesh_to_assimp_lib
  src/pcl_mesh_to_assimp.cpp
)

# Set include directories for the library itself
target_include_directories(pcl_mesh_to_assimp_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  # PCL and Assimp headers are implicitly handled via target_link_libraries
)

# Link the utility library to its direct dependencies (PCL and Assimp)
target_link_libraries(pcl_mesh_to_assimp_lib
  ${PCL_LIBRARIES}
  assimp::assimp # Correct way to link Assimp from find_package
)

# Include the custom srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Room.msg"
  "msg/RoomObject.msg"
  "srv/ListRooms.srv"
  "srv/ListRoomObjects.srv"
  "srv/MoveToObject.srv"
)

add_executable(dsg_query_node src/dsg_query_node.cpp)
add_executable(dsg_map_node src/dsg_map_node.cpp)

target_link_libraries(dsg_map_node
  pcl_mesh_to_assimp_lib
  ${PCL_LIBRARIES}
  assimp::assimp
)

ament_target_dependencies(dsg_query_node
  rclcpp
  hydra_msgs
  spark_dsg
)
ament_target_dependencies(dsg_map_node
  rclcpp
  hydra_msgs
  pose_graph_tools_msgs
  spark_dsg
  std_srvs
  kimera_pgmo_ros
  nav2_msgs
  geometry_msgs
)

target_include_directories(dsg_map_node PRIVATE ${PCL_INCLUDE_DIRS})


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

rosidl_target_interfaces(dsg_map_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS 
  dsg_query_node
  dsg_map_node
  pcl_mesh_to_assimp_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h"
)

ament_package()
