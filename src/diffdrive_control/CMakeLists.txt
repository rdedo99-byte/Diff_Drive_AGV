cmake_minimum_required(VERSION 3.16)
project(diffdrive_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(pal_statistics REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/diff_drive_esp32_hw_interface.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  pluginlib
  std_msgs
  pal_statistics
)

pluginlib_export_plugin_description_file(hardware_interface diff_drive_esp32_hw_interface.xml)
install(FILES diff_drive_esp32_hw_interface.xml DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

# RIMOZIONE: Rimossa installazione di config/diff_drive_controller.yaml

# Installazione dei file di lancio (launch) - Solo per il launch file refactorizzato
# NUOVO BLOCCO: Installazione esplicita del file di lancio
install(
  FILES launch/diff_drive.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

add_executable(plugin_test src/plugin_test.cpp)
ament_target_dependencies(plugin_test pluginlib hardware_interface)
target_link_libraries(plugin_test pluginlib::pluginlib)

install(TARGETS plugin_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  pluginlib
  std_msgs
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()