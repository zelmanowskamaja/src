cmake_minimum_required(VERSION 3.5)
project(navigation)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tello_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

add_executable(cpp_executable src/${PROJECT_NAME}.cpp src/${PROJECT_NAME}_node.cpp)
target_include_directories(cpp_executable PUBLIC include)
ament_target_dependencies(cpp_executable rclcpp geometry_msgs tello_msgs tf2_ros gazebo_msgs)

install(TARGETS
  cpp_executable
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch src scripts DESTINATION share/${PROJECT_NAME})

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

# Install Python modules
# ament_python_install_package(scripts)
ament_python_install_package(${PROJECT_NAME})

# install(DIRECTORY
#   launch
#   src
#   DESTINATION install/${PROJECT_NAME}
# )
# install(DIRECTORY
#   scripts
#   DESTINATION install/${PROJECT_NAME}
# )
# Install Python executables
install(PROGRAMS
  scripts/publisher.py
  DESTINATION lib/${PROJECT_NAME}/
)


# Export package dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_include_directories(include ${rclcpp_INCLUDE_DIRS})

ament_package()