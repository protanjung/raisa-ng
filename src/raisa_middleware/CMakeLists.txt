cmake_minimum_required(VERSION 3.8)
project(raisa_middleware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(var_dependencies
    nav_msgs
    pandu_ros2_kit
    pcl_conversions
    pcl_ros
    raisa_interfaces
    rclcpp
    std_srvs
    tf2_geometry_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
foreach(dependency ${var_dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(obstacle_detector src/obstacle_detector.cpp)
add_executable(pose_estimator src/pose_estimator.cpp)
add_executable(transform_broadcaster src/transform_broadcaster.cpp)
ament_target_dependencies(obstacle_detector ${var_dependencies})
ament_target_dependencies(pose_estimator ${var_dependencies})
ament_target_dependencies(transform_broadcaster ${var_dependencies})
target_link_libraries(obstacle_detector ${PCL_LIBRARIES})

install(TARGETS obstacle_detector pose_estimator transform_broadcaster
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
