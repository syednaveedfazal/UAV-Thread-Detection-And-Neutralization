cmake_minimum_required(VERSION 3.8)
project(poc1_landing_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install the directories so ROS can find them at runtime
install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()