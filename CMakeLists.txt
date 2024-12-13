cmake_minimum_required(VERSION 3.8)
project(trac_ik_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
#find_package(kdl REQUIRED)

add_executable(start_ik_service src/start_ik_service.cpp)
target_include_directories(start_ik_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(start_ik_service PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(start_ik_service
  "rclcpp"
  "trac_ik_lib"
  moveit_msgs
  tf2_kdl
#  "kdl"
)

install(TARGETS start_ik_service
  DESTINATION lib/${PROJECT_NAME})


add_executable(test_ik_service src/ik_tests.cpp)
target_include_directories(test_ik_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_ik_service PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(test_ik_service
  "rclcpp"
  "trac_ik_lib"
  moveit_msgs
  tf2_kdl
#  "kdl"
)

install(TARGETS test_ik_service
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
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
