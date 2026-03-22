cmake_minimum_required(VERSION 3.8)
project(stm32_robot_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# Serial Bridge Node
add_executable(serial_bridge_node src/serial_bridge_node.cpp)
target_include_directories(serial_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(serial_bridge_node PUBLIC c_std_99 cxx_std_17)  
ament_target_dependencies(serial_bridge_node rclcpp std_msgs geometry_msgs)
rosidl_target_interfaces(serial_bridge_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Teleop Keyboard Node
add_executable(teleop_keyboard_node src/teleop_keyboard_node.cpp)
target_include_directories(teleop_keyboard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(teleop_keyboard_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(teleop_keyboard_node rclcpp std_msgs geometry_msgs)
rosidl_target_interfaces(teleop_keyboard_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  serial_bridge_node
  teleop_keyboard_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config msg
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
