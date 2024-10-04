cmake_minimum_required(VERSION 3.8)
project(montemorency)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)

add_executable(montemorency_pcl 
    src/montemorency_pcl.cpp
)

target_include_directories(montemorency_pcl PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
)

target_link_libraries(montemorency_pcl PUBLIC 
  rclcpp::rclcpp 
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${PCL_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

include_directories( include ${PCL_INCLUDE_DIRS} )

install (TARGETS montemorency_pcl DESTINATION lib/${PROJECT_NAME})

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
