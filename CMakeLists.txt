cmake_minimum_required(VERSION 3.8)
project(attack_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
#find_package(ament_cmake_gtest REQUIRED)

add_executable(hfr_simulator src/hfr_simulator.cpp )
add_executable(utils src/utils/velocity_indicator.cpp)



target_include_directories(hfr_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  target_include_directories(utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hfr_simulator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(utils PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hfr_simulator
  "rclcpp"
  "rclpy"
  "ament_cmake_python"
  "std_msgs"
  "sensor_msgs"
  "autoware_vehicle_msgs"
)
ament_target_dependencies(
  utils
  "rclcpp"
  "rclpy"
  "ament_cmake_python"
  "std_msgs"
  "sensor_msgs"
  "autoware_vehicle_msgs"
  
)

target_link_libraries(hfr_simulator Eigen3::Eigen)


install(TARGETS hfr_simulator
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS utils
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# ament_add_gtest(test_hfr_simulator test/test_hfr_simulator.cpp)
# target_include_directories(test_hfr_simulator PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(test_hfr_simulator
#   "rclcpp"
#   "std_msgs"
#   "sensor_msgs"
# )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
