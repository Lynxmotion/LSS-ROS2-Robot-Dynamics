cmake_minimum_required(VERSION 3.5)
project(humanoid_dynamic_model)

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

# Remove -Wdeprecated-copy GCC warnings which generate a lots of compile warnings
if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-copy")
endif()

# external dependancies
find_package(PCL REQUIRED common geometry surface)

# find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rmw REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(humanoid_model_msgs REQUIRED)

add_subdirectory(RobotIK)

add_executable(legacy_robot_dynamics
        legacy_robot_dynamics/robot_dynamics.h legacy_robot_dynamics/robot_dynamics.cpp
        legacy_robot_dynamics/default_main.cpp
        )
target_compile_definitions(robot_dynamics PRIVATE NODE_CLASS_TYPE=RobotDynamics NODE_CLASS_INCLUDE="robot_dynamics.h")
target_include_directories(robot_dynamics PUBLIC
        ${ROBOTIK_INCLUDE_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(robot_dynamics
        robotik-objlib
        ${humanoid_model_msgs_LIBRARIES}
        ${PCL_LIBRARIES})
ament_target_dependencies(robot_dynamics
        "rclcpp"
        "urdf"
        "urdfdom_headers"
        "tf2"
        "tf2_kdl"
        "tf2_ros"
        "diagnostic_msgs"
        "lifecycle_msgs"
        "sensor_msgs"
        "std_srvs"
        "nav_msgs"
        "trajectory_msgs"
        "visualization_msgs"
        "interactive_markers"
        "humanoid_model_msgs"
        "rclcpp_components"
        "rclcpp_lifecycle"
        "orocos_kdl"
        "kdl_parser")


install(TARGETS robot_dynamics
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
