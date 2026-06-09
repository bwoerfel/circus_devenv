cmake_minimum_required(VERSION 3.8)
project(ca1_motor_ctrl)

# Require C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -------------------------------------------------------------------------
# Service interfaces
# -------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/OdRead.srv"
  "srv/OdWrite.srv"
)

# Allow linking against generated interfaces within the same package.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# -------------------------------------------------------------------------
# Simulator library  (reused by the node, the tests, and the user's node)
# -------------------------------------------------------------------------
add_library(drive_simulator
  src/drive_simulator.cpp
)

target_include_directories(drive_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# No ROS dependencies in the library – keeps it portable and easy to test.
target_compile_features(drive_simulator PUBLIC cxx_std_17)

# -------------------------------------------------------------------------
# Simulator ROS 2 node
# -------------------------------------------------------------------------
add_executable(drive_simulator_node
  src/drive_simulator_node.cpp
)

target_include_directories(drive_simulator_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drive_simulator_node
  drive_simulator
  "${cpp_typesupport_target}"
)

ament_target_dependencies(drive_simulator_node rclcpp std_msgs std_srvs)

# -------------------------------------------------------------------------
# Drive controller library  (pure C++, no ROS 2 – mirrors drive_simulator)
# -------------------------------------------------------------------------
add_library(drive_controller
  src/drive_controller.cpp
)

target_include_directories(drive_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(drive_controller
  drive_simulator
)

target_compile_features(drive_controller PUBLIC cxx_std_17)

# -------------------------------------------------------------------------
# Drive controller ROS 2 node
# -------------------------------------------------------------------------
add_executable(drive_controller_node
  src/drive_controller_node.cpp
)

target_include_directories(drive_controller_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drive_controller_node
  drive_controller
  "${cpp_typesupport_target}"
)

ament_target_dependencies(drive_controller_node rclcpp std_msgs std_srvs)

# -------------------------------------------------------------------------
# Unit tests
# -------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_drive_simulator
    test/test_drive_simulator.cpp
  )

  target_include_directories(test_drive_simulator PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_drive_simulator
    drive_simulator
  )

  ament_add_gtest(test_drive_controller
    test/test_drive_controller.cpp
  )

  target_include_directories(test_drive_controller PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_drive_controller
    drive_controller
  )
endif()

# -------------------------------------------------------------------------
# Install
# -------------------------------------------------------------------------
install(TARGETS
  drive_simulator
  drive_simulator_node
  drive_controller
  drive_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS src/web_ui.py
  RENAME web_ui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(drive_simulator drive_controller)
ament_package()
