cmake_minimum_required(VERSION 3.15)
project(multiscan_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT MSVC)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(include)
include_directories(SYSTEM)

add_library(scansegment_xd STATIC
  "src/sick_scan_xd/msgpack11/msgpack11.cpp"
  "src/sick_scan_xd/compact_parser.cpp"
  "src/sick_scan_xd/msgpack_parser.cpp"
  "src/sick_scan_xd/scansegment_parser_output.cpp"
  "src/sick_scan_xd/softwarePLL.cpp"
  "src/sick_scan_xd/udp_receiver.cpp"
  "src/sick_scan_xd/udp_sockets.cpp"
  "src/sick_scan_xd/wsa_init.cpp")
ament_target_dependencies(scansegment_xd)

add_executable(multiscan_driver "src/multiscan_driver.cpp")
target_link_libraries(multiscan_driver
  scansegment_xd
  Eigen3::Eigen)
ament_target_dependencies(multiscan_driver
  rclcpp
  std_msgs
  sensor_msgs)
target_include_directories(multiscan_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(multiscan_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS multiscan_driver
  DESTINATION lib/${PROJECT_NAME})

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

ament_export_dependencies(rclcpp std_msgs sensor_msgs)
ament_package()
