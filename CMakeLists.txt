cmake_minimum_required(VERSION 3.8)
project(pfcbf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set environment variables
set(OsqpEigen_DIR /home/matti/lib/cmake/OsqpEigen/)
set(osqp_DIR /usr/local/lib/cmake/osqp/)

set(CMAKE_INSTALL_RPATH "/home/matti/lib/")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(
  include
  ${OsqpEigen_DIR}
  ${osqp_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  nav2_msgs
  std_msgs
  visualization_msgs
  tf2
  geometry_msgs
  OsqpEigen
  Eigen3
  tf2_geometry_msgs
  std_srvs
)

add_executable(pfcbf_node
  src/pfcbf_node.cpp
  src/pfcbf.cpp
  include/helper_functions.h
  include/pfcbf.h
  src/helper_functions.cpp  
)

target_link_libraries(pfcbf_node OsqpEigen::OsqpEigen)

ament_target_dependencies(pfcbf_node ${dependencies})


install(TARGETS pfcbf_node RUNTIME DESTINATION lib/pfcbf)

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
