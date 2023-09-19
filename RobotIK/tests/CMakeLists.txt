
set(TEST_FILES trajectory.cpp interpolate.cpp graph.cpp)

add_executable(robot_dynamics_tests
       ${TEST_FILES}
        main.cpp
        )
target_compile_features(robot_dynamics_tests PRIVATE cxx_std_14)
target_include_directories(robot_dynamics_tests PUBLIC
        ${ROBOTIK_INCLUDE_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(robot_dynamics_tests
        robotik-objlib
        ${humanoid_model_msgs_LIBRARIES}
        ${PCL_LIBRARIES})
ament_target_dependencies(robot_dynamics_tests
        "rclcpp"
        "urdf"
        "urdfdom_headers"
        "tf2"
        "tf2_kdl"
        "tf2_ros"
        "diagnostic_msgs"
        "lifecycle_msgs"
        "sensor_msgs"
        "nav_msgs"
        "trajectory_msgs"
        "visualization_msgs"
        "interactive_markers"
        "humanoid_model_msgs"
        "rclcpp_components"
        "rclcpp_lifecycle"
        "orocos_kdl"
        "kdl_parser")

install(TARGETS robot_dynamics_tests
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})
