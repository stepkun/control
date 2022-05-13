cmake_minimum_required(VERSION 3.8)
project(nodebot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include C++ "include" directory
#include_directories(include)

# Create C++ executable
#add_executable(control_cpp src/control.cpp)
#ament_target_dependencies(control_cpp rclcpp geometry_msgs std_msgs)

# Install C++ executables
#install(
#  TARGETS control_cpp
#  DESTINATION lib/${PROJECT_NAME}
#)

# Install Python modules
ament_python_install_package(nodebot_control)

# Install Python executables as scripts
install(PROGRAMS
  scripts/app scripts/two_joystick_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
