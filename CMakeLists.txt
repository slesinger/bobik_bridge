cmake_minimum_required(VERSION 3.5)
project(bobik_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(bobik_bridge src/bobik_bridge.cpp)

# target_include_directories(bobik_bridge PUBLIC /usr/local/include)
target_link_libraries(bobik_bridge /usr/local/lib/libzmq.so)

ament_target_dependencies(
  bobik_bridge
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
bobik_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
