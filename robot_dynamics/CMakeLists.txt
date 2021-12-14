
add_library(robot_dynamics SHARED
        robot_dynamics.h robot_dynamics.cpp)

target_include_directories(robot_dynamics PUBLIC
        ${ROBOTIK_INCLUDE_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
link_directories(${PCL_LIBRARY_DIRS})
add_dependencies(robot_dynamics robotik-objlib)
target_link_libraries(robot_dynamics
        robotik-objlib
        ${humanoid_model_msgs_LIBRARIES}
        ${PCL_LIBRARIES})

# todo: cleanup these deps
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
        "humanoid_model_msgs"
        "rclcpp_components"
        "rclcpp_lifecycle"
        "orocos_kdl"
        "kdl_parser")

ament_export_libraries(robot_dynamics)

install(TARGETS robot_dynamics
        EXPORT export_${PROJECT_NAME}
        #DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

