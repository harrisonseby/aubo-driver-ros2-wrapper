cmake_minimum_required(VERSION 3.8)
project(aubo_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Select prebuilt SDK library based on pointer size (32 vs 64-bit)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(AUBO_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64)
else()
  set(AUBO_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib32)
endif()

file(GLOB AUBO_RUNTIME_LIBS "${AUBO_LIB_DIR}/libauborobotcontroller.so*")

include_directories(
  include
  include/aubo_driver/otg
)

# Build the hardware_interface plugin as a shared library
add_library(${PROJECT_NAME} SHARED
  src/aubo_driver.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  pthread
  ${AUBO_LIB_DIR}/libauborobotcontroller.so
  ${AUBO_LIB_DIR}/libotgLib.a
)

# Register the pluginlib plugin so controller_manager can discover it
pluginlib_export_plugin_description_file(hardware_interface aubo_driver_plugin.xml)

# Build a minimal standalone bringup node (for testing without ros2_control)
add_executable(aubo_driver_node src/driver_node.cpp)
ament_target_dependencies(aubo_driver_node rclcpp)

# Install targets
install(TARGETS ${PROJECT_NAME} aubo_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install all SDK shared library filenames so the dynamic loader can resolve
# the library SONAME at runtime.
install(FILES
  ${AUBO_RUNTIME_LIBS}
  DESTINATION lib
)

# Install config files (SDK reads these from the working directory at runtime)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
