
set(INCLUDE_FILES
        robot_control.h
        tree-kinematics.h
        trajectory/path-rounded-composite.h
        trajectory/treeiksolverpos_online.hpp
        trajectory/environment.h
        trajectory/expression.h
        trajectory/rendering.h
        trajectory/action.h
        trajectory/single-effector.h
        trajectory/linear-trajectory.h
        trajectory/coordinated-effectors.h)
set(SOURCE_FILES
        robot_control.cpp
        tree-kinematics.cpp
        trajectory/path-rounded-composite.cpp
        trajectory/treeiksolverpos_online.cpp
        trajectory/expression.cpp
        trajectory/rendering.cpp
        trajectory/action.cpp
        trajectory/linear-trajectory.cpp
        trajectory/single-effector.cpp
        trajectory/coordinated-effectors.cpp)

list(TRANSFORM INCLUDE_FILES PREPEND include/robot_control/)
list(TRANSFORM SOURCE_FILES PREPEND src/)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

add_library(robot_control SHARED ${INCLUDE_FILES} ${SOURCE_FILES})

target_include_directories(robot_control PUBLIC
        ${ROBOTIK_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
link_directories(${PCL_LIBRARY_DIRS})
add_dependencies(robot_control robotik-objlib)
target_link_libraries(robot_control
        robotik-objlib
        ${robot_model_msgs_LIBRARIES}
        ${PCL_LIBRARIES})

# todo: cleanup these deps
ament_target_dependencies(robot_control
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
        "trajectory_msgs"
        "robot_model_msgs"
        "rclcpp_components"
        "rclcpp_lifecycle"
        "orocos_kdl"
        "kdl_parser")


install(TARGETS robot_control
        EXPORT export_${PROJECT_NAME}
        #DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

