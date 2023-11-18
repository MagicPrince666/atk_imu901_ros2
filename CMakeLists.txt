cmake_minimum_required(VERSION 3.8)
project(ros2_imu)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_BUILD_TYPE "Debug")
if (${CMAKE_BUILD_TYPE} MATCHES "Debug")
  add_definitions(-DBACKTRACE_DEBUG)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
    ${roscpp_INCLUDE_DIRS} 
    ${roslib_INCLUDE_DIRS} 
    include
    src/drivers
    src/mpu6050
    src/mpu9250
)

file(GLOB main_srcs 
    src/*.cpp
    src/drivers/*.cpp
    src/mpu6050/*.cpp
    src/mpu9250/*.cpp
)

add_executable(${PROJECT_NAME}_node ${main_srcs})
target_link_libraries(${PROJECT_NAME}_node pthread)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
		DESTINATION share/${PROJECT_NAME}
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
