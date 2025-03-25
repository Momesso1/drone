cmake_minimum_required(VERSION 3.5)
project(autonomous_map)

set(CMAKE_CXX_STANDARD 20)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(yolo_msgs REQUIRED)
find_library(XXHASH_LIBRARY xxhash REQUIRED)

add_executable(create_graph src/create_graph.cpp)
ament_target_dependencies(create_graph rclcpp std_msgs nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs)

add_executable(docking_node src/docking_node.cpp)
ament_target_dependencies(docking_node rclcpp std_msgs nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs tf2_geometry_msgs)

add_executable(a_star src/a_star.cpp)
ament_target_dependencies(a_star rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )

add_executable(visualize_a_star src/visualize_a_star.cpp)
ament_target_dependencies(visualize_a_star rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )

add_executable(visualize_obstacles_graph src/visualize_obstacles_graph.cpp)
ament_target_dependencies(visualize_obstacles_graph rclcpp std_msgs nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs)

add_executable(bidirecional_a_star src/bidirecional_a_star.cpp)
ament_target_dependencies(bidirecional_a_star rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )

add_executable(send_poses src/send_poses.cpp)
ament_target_dependencies(send_poses rclcpp nav_msgs geometry_msgs yaml-cpp PCL pcl_conversions nav_msgs tf2_geometry_msgs sensor_msgs yolo_msgs)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp nav_msgs geometry_msgs tf2_geometry_msgs)

add_executable(save_point_cloud src/save_point_cloud.cpp)
ament_target_dependencies(save_point_cloud rclcpp std_msgs nav_msgs geometry_msgs PCL pcl_conversions)


target_link_libraries(create_graph
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)


target_link_libraries(docking_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)


target_link_libraries(a_star
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(bidirecional_a_star
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)


target_link_libraries(visualize_a_star
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(visualize_obstacles_graph
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)


target_link_libraries(send_poses
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

target_link_libraries(controller
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(save_point_cloud
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(
  DIRECTORY
    launch

DESTINATION share/${PROJECT_NAME}
)


install(TARGETS
  create_graph
  docking_node
  send_poses
  a_star
  bidirecional_a_star
  controller
  visualize_a_star
  visualize_obstacles_graph
  save_point_cloud
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

