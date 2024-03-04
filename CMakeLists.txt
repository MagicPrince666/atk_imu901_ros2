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

message(STATUS "${LIB_NAME} Ros version : $ENV{ROS_DISTRO}")
if ($ENV{ROS_DISTRO} MATCHES "iron")
  add_definitions(-DUSE_IRON_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
  find_package(spdlog REQUIRED)
  set(LIBSPDLOG spdlog::spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "humble")
  add_definitions(-DUSE_HUMBLE_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
  find_package(spdlog REQUIRED)
  set(LIBSPDLOG spdlog::spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "galactic")
  add_definitions(-DUSE_GALACTIC_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
  find_package(spdlog REQUIRED)
  set(LIBSPDLOG spdlog::spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "foxy")
  add_definitions(-DUSE_FOXY_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
  find_package(spdlog REQUIRED)
  set(LIBSPDLOG spdlog::spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "eloquent")
  add_definitions(-DUSE_ELOQUENT_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
  include_directories(include/spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "dashing")
  add_definitions(-DUSE_DASHING_VERSION)
  set(USE_ROS1_MIDDLE_PRICE OFF)
  include_directories(include/spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "noetic")
  add_definitions(-DUSE_ROS_NORTIC_VERSION)
  set(USE_ROS1_MIDDLE_PRICE ON)
  include_directories(include/spdlog)
elseif ($ENV{ROS_DISTRO} MATCHES "melodic")
  add_definitions(-DUSE_ROS_MELODIC_VERSION)
  set(USE_ROS1_MIDDLE_PRICE ON)
  include_directories(include/spdlog)
endif()

include_directories(
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

if(USE_ROS1_MIDDLE_PRICE)
######################## 使用ROS1 作为中间件 ###########################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

add_executable(${PROJECT_NAME}_node ${main_srcs})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS}) 

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} pthread ${LIBSPDLOG})

install(DIRECTORY ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#######################################################################
else()
######################## 使用ROS2 作为中间件 ###########################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
    ${roscpp_INCLUDE_DIRS}
    ${roslib_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${main_srcs})
target_link_libraries(${PROJECT_NAME}_node pthread ${LIBSPDLOG})
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
#######################################################################
endif()