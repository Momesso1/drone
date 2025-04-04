cmake_minimum_required(VERSION 3.5)
project(navigation_2d)

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
find_package(visualization_msgs REQUIRED) 
add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp nav_msgs geometry_msgs tf2_geometry_msgs)

add_executable(create_graph src/create_graph.cpp)
ament_target_dependencies(create_graph rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )

add_executable(2d_a_star_with_filter src/2d_a_star_with_filter.cpp)
ament_target_dependencies(2d_a_star_with_filter rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )

add_executable(2d_bidirectional_a_star_with_filter src/2d_bidirectional_a_star_with_filter.cpp)
ament_target_dependencies(2d_bidirectional_a_star_with_filter rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )

add_executable(rrt_with_filter src/rrt_with_filter.cpp)
ament_target_dependencies(rrt_with_filter rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs visualization_msgs)


target_link_libraries(controller
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(create_graph
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(2d_a_star_with_filter
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(2d_bidirectional_a_star_with_filter
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(rrt_with_filter
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)



install(
 
  DIRECTORY
  rviz
  launch
DESTINATION share/${PROJECT_NAME}
)


install(TARGETS
  controller
  create_graph
  2d_a_star_with_filter
  2d_bidirectional_a_star_with_filter
  rrt_with_filter
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

