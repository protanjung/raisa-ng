cmake_minimum_required(VERSION 3.8)
project(raisa_io)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(var_dependencies
    cv_bridge
    pandu_ros2_kit
    pcl_conversions
    raisa_interfaces
    rclcpp
    rclpy)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
foreach(dependency ${var_dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(io_basestation src/io_basestation.cpp)
add_executable(io_lslidar_n301 src/io_lslidar_n301.cpp)
add_executable(io_stm32 src/io_stm32.cpp)
add_executable(io_ui src/io_ui.cpp)
add_executable(io_vision src/io_vision.cpp)
ament_target_dependencies(io_basestation ${var_dependencies})
ament_target_dependencies(io_lslidar_n301 ${var_dependencies})
ament_target_dependencies(io_stm32 ${var_dependencies})
ament_target_dependencies(io_ui ${var_dependencies})
ament_target_dependencies(io_vision ${var_dependencies})
target_link_libraries(io_lslidar_n301 ${PCL_LIBRARIES})
target_link_libraries(io_vision ${OpenCV_LIBRARIES})

install(
  TARGETS io_basestation
          io_lslidar_n301
          io_stm32
          io_ui
          io_vision
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/io_thermal.py scripts/io_mb1r2t.py
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
