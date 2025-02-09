cmake_minimum_required(VERSION 3.12.2)
project(bluerov2_fortress)

# Definir padrão C++ se não especificado
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Configurações adicionais para compiladores específicos
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar pacotes necessários
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

# Localizar dependências automaticamente
ament_auto_find_build_dependencies()

# Executáveis
ament_auto_add_executable(camera_info_publisher
  src/camera_info_publisher.cpp
)

ament_auto_add_executable(camera_sync_publisher
  src/camera_sync_publisher.cpp
)

ament_auto_add_executable(teleop_keyboard
  src/teleop_keyboard.cpp
)

ament_target_dependencies(camera_sync_publisher
  rclcpp
  sensor_msgs
  message_filters
)

ament_target_dependencies(camera_sync_publisher
  rclcpp
  std_msgs
 
)
# Instalação de arquivos compartilhados
install(
  DIRECTORY launch models worlds config
  DESTINATION share/${PROJECT_NAME}
)

# Finalizar pacotes
ament_auto_package()
