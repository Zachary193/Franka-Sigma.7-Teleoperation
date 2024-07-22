cmake_minimum_required(VERSION 3.0.2)
project(franka_execute)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  actionlib
  actionlib_msgs
  franka_gripper
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp moveit_ros_planning_interface geometry_msgs tf2_ros actionlib actionlib_msgs franka_gripper
)

# Declare C++ executable
add_executable(franka_execute src/franka_execute.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(franka_execute
  ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS franka_execute
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
