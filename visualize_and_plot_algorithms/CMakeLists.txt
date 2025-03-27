cmake_minimum_required(VERSION 3.5)
project(visualize_and_plot_algorithms)

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


add_executable(visualize_a_star src/visualize_a_star.cpp)
ament_target_dependencies(visualize_a_star rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )

add_executable(visualize_a_star_with_filter src/visualize_a_star_with_filter.cpp)
ament_target_dependencies(visualize_a_star_with_filter rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )

add_executable(visualize_obstacles_graph src/visualize_obstacles_graph.cpp)
ament_target_dependencies(visualize_obstacles_graph rclcpp std_msgs nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs)

add_executable(visualize_bidirectional_a_star src/visualize_bidirectional_a_star.cpp)
ament_target_dependencies(visualize_bidirectional_a_star rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )

add_executable(visualize_bidirectional_a_star_with_filter src/visualize_bidirectional_a_star_with_filter.cpp)
ament_target_dependencies(visualize_bidirectional_a_star_with_filter rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )


add_executable(plot_a_star src/plot_a_star.cpp)
ament_target_dependencies(plot_a_star rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )

add_executable(plot_bidirectional_a_star src/plot_bidirectional_a_star.cpp)
ament_target_dependencies(plot_bidirectional_a_star rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )




target_link_libraries(visualize_a_star
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(visualize_a_star_with_filter
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)


target_link_libraries(visualize_bidirectional_a_star
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(visualize_bidirectional_a_star_with_filter
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

target_link_libraries(plot_a_star
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)


target_link_libraries(plot_bidirectional_a_star
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)




install(
  DIRECTORY
    launch
    rviz
DESTINATION share/${PROJECT_NAME}
)


install(TARGETS
  visualize_a_star
  visualize_a_star_with_filter
  visualize_bidirectional_a_star
  visualize_bidirectional_a_star_with_filter
  visualize_obstacles_graph
  plot_a_star
  plot_bidirectional_a_star
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

