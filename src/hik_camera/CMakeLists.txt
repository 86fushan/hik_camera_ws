cmake_minimum_required(VERSION 3.8)
project(hik_camera)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
set(MVS_INCLUDE_DIR "/opt/MVS/include")
set(MVS_LIBRARY "/opt/MVS/lib/64/libMvCameraControl.so")
if(NOT EXISTS "${MVS_INCLUDE_DIR}/MvCameraControl.h")
  message(FATAL_ERROR "MVS header not found: ${MVS_INCLUDE_DIR}/MvCameraControl.h")
endif()

if(NOT EXISTS "${MVS_LIBRARY}")
  message(FATAL_ERROR "MVS library not found: ${MVS_LIBRARY}")
endif()

add_executable(hik_camera_node
  src/hik_camera_node.cpp
)

target_include_directories(hik_camera_node PRIVATE
  ${MVS_INCLUDE_DIR}
)

ament_target_dependencies(hik_camera_node
  rclcpp
  sensor_msgs
)

target_link_libraries(hik_camera_node
  ${MVS_LIBRARY}
)
set_target_properties(hik_camera_node PROPERTIES
  BUILD_RPATH "/opt/MVS/lib/64"
  INSTALL_RPATH "/opt/MVS/lib/64"
)
install(
  TARGETS hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
