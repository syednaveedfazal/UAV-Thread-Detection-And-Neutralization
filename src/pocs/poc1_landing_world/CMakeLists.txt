cmake_minimum_required(VERSION 3.8)
project(poc1_landing_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Models
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# Install Worlds
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# --- CRITICAL PART: INSTALL THE PYTHON SCRIPT ---
# This copies the file to lib/poc1_landing_world/ so 'ros2 run' finds it
install(PROGRAMS
  src/mission_control.py
  DESTINATION lib/${PROJECT_NAME}
)
# ------------------------------------------------

ament_package()