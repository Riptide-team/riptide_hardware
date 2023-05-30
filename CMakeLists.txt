cmake_minimum_required(VERSION 3.8)
project(riptide_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# External libraries driver
include(FetchContent) # once in the project to include the module
FetchContent_Declare(sparton_driver GIT_REPOSITORY https://github.com/Teusner-Thesis/Sparton_AHRS_M1_Driver.git GIT_TAG main)
FetchContent_MakeAvailable(sparton_driver)

FetchContent_Declare(sea_scan_echo_driver GIT_REPOSITORY https://github.com/Teusner/Sea_Scan_Echo_Marine_Sonic_Driver.git GIT_TAG main)
FetchContent_MakeAvailable(sea_scan_echo_driver)

FetchContent_Declare(ms5837_driver GIT_REPOSITORY https://github.com/Teusner/MS5837_Driver.git GIT_TAG main)
FetchContent_MakeAvailable(ms5837_driver)

FetchContent_Declare(rtac_asio GIT_REPOSITORY https://github.com/pnarvor/rtac_asio.git GIT_TAG master)
FetchContent_MakeAvailable(rtac_asio)

FetchContent_Declare(NemaTode GIT_REPOSITORY https://github.com/ckgt/NemaTode.git GIT_TAG master)
FetchContent_MakeAvailable(NemaTode)


# Compile riptide_hardware
add_library(${PROJECT_NAME} SHARED src/imu_hardware.cpp src/pressure_hardware.cpp src/echosounder_hardware.cpp src/tail_hardware.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} sparton_ahrs_m1_driver sea_scan_echo_driver ms5837_driver rtac_asio NemaTode)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface riptide_hardware.xml)

# INSTALL
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  # ament_add_gmock(test_sparton_ahrs_m1_hwi test/test_sparton_ahrs_m1_hwi.cpp)
  # target_include_directories(test_sparton_ahrs_m1_hwi PRIVATE include)
  # target_link_libraries(test_sparton_ahrs_m1_hwi sparton_ahrs_m1_hardware_interface)
  # ament_target_dependencies(test_sparton_ahrs_m1_hwi
  #   pluginlib
  #   ros2_control_test_assets
  # )
endif()

## EXPORTS
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sparton_m1_driver
  sea_scan_echo_driver
  ms5837_driver
  rtac_asio
)
ament_package()
