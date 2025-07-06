cmake_minimum_required(VERSION 3.8)
project(livox_sdk2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Link against Livox SDK
find_library(LIVOX_LIDAR_SDK_STATIC livox_lidar_sdk_static PATHS /usr/local/lib)
find_library(LIVOX_LIDAR_SDK_SHARED livox_lidar_sdk_shared PATHS /usr/local/lib)

add_executable(livox_mid360 src/main.cpp src/livox_mid360.cpp src/filters/madgwick.cpp)
target_include_directories(livox_mid360 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include
)
target_compile_features(livox_mid360 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  livox_mid360
  "rclcpp"
  "sensor_msgs"
  "Eigen3"
  "tf2"
  "tf2_ros"
)

target_link_libraries(livox_mid360
  ${LIVOX_LIDAR_SDK_SHARED}
)

install(TARGETS livox_mid360
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

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
