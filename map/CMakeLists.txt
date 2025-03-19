cmake_minimum_required(VERSION 3.5)
project(map)

# Default to C++14
set(CMAKE_CXX_STANDARD 17)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(create_graph src/create_graph.cpp)
ament_target_dependencies(create_graph rclcpp std_msgs nav_msgs geometry_msgs PCL pcl_conversions)

add_executable(verify_path src/verify_path.cpp)
ament_target_dependencies(verify_path rclcpp std_msgs nav_msgs geometry_msgs PCL pcl_conversions)


add_executable(a_estrela src/a_estrela.cpp)
ament_target_dependencies(a_estrela rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )



target_link_libraries(create_graph
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(verify_path
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
)




target_link_libraries(a_estrela
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
)



install(
  DIRECTORY
    launch

  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS
  create_graph
  a_estrela
  verify_path
  
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()