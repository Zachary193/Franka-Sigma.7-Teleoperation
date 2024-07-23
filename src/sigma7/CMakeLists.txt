cmake_minimum_required(VERSION 3.0.2)
project(sigma7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs tf tf2_ros tf2_geometry_msgs
)

# Declare a C++ executable for the main program
add_executable(sigma7 src/sigma7.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(sigma7
  ${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdhd.so.3
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdrd.so.3
)

# Install targets and directories
install(TARGETS sigma7
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
