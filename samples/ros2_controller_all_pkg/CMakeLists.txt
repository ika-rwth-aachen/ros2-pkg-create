cmake_minimum_required(VERSION 3.8)
project(ros2_controller_all_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ros2_controller_all_pkg_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)  

add_library(${PROJECT_NAME} SHARED
  src/custom_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PUBLIC ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(${PROJECT_NAME}
  PUBLIC c_std_99 # Require C99
  PUBLIC cxx_std_17 # Require C++17
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  ros2_controller_all_pkg_interfaces
)

pluginlib_export_plugin_description_file(
  controller_interface ros2_controller_all_pkg.xml
)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # Test to see if controllers can be loaded
  ament_add_gmock(${PROJECT_NAME}_test_load
    test/test_load_custom_controller.cpp
  )
  target_include_directories(${PROJECT_NAME}_test_load
    PRIVATE include
  )
  ament_target_dependencies(${PROJECT_NAME}_test_load
    controller_manager
    ros2_control_test_assets
  )

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Also install the include folder contents and the generated library
ament_export_include_directories(
    include
)
ament_export_libraries(
    ${PROJECT_NAME}
)

ament_package()
