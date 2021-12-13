
add_executable(legacy_robot_dynamics
        robot_dynamics.h robot_dynamics.cpp
        default_main.cpp
        )
target_compile_definitions(legacy_robot_dynamics PRIVATE NODE_CLASS_TYPE=RobotDynamics NODE_CLASS_INCLUDE="robot_dynamics.h")
target_include_directories(legacy_robot_dynamics PUBLIC
        ${ROBOTIK_INCLUDE_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
link_directories(${PCL_LIBRARY_DIRS})
add_dependencies(legacy_robot_dynamics robotik-objlib)
target_link_libraries(legacy_robot_dynamics
        robotik-objlib
        ${humanoid_model_msgs_LIBRARIES}
        ${PCL_LIBRARIES})
ament_target_dependencies(legacy_robot_dynamics
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


install(TARGETS legacy_robot_dynamics
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

