cmake_minimum_required(VERSION 3.8)
project(yur_ros2_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# include directories
include_directories(include)

# Create library
set(${PROJECT_NAME}_SOURCES
  src/yur_ros_wrapper.cpp
  src/ur_driver.cpp
  src/ur_realtime_communication.cpp
  src/ur_communication.cpp
  src/robot_state.cpp
  src/robot_state_RT.cpp
  src/do_output.cpp
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
# For propagating the header for library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# For propagating the required compiler for library
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
# Dependencies for library 
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
)

# Create executables
add_executable(yur_single src/singleArm_bringup.cpp)
target_link_libraries(yur_single ${PROJECT_NAME})
ament_target_dependencies(yur_single
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(yur_multi src/multiArm_bringup.cpp)
target_link_libraries(yur_multi ${PROJECT_NAME})
ament_target_dependencies(yur_multi
  rclcpp
  std_msgs
  sensor_msgs
)

# Install targets

# Install library target
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install execution target
install(TARGETS yur_single yur_multi
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files, config files, etc. (if any)
# install(DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME}/
# )

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()