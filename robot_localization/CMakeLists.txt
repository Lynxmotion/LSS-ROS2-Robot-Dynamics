
set(INCLUDE_FILES
        robot_localization.h)
set(SOURCE_FILES
        robot_localization.cpp)

list(TRANSFORM INCLUDE_FILES PREPEND include/robot_localization/)
list(TRANSFORM SOURCE_FILES PREPEND src/)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

add_library(robot_localization SHARED ${INCLUDE_FILES} ${SOURCE_FILES})

target_include_directories(robot_localization PUBLIC
        ${ROBOTIK_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
link_directories(${PCL_LIBRARY_DIRS})
add_dependencies(robot_localization robotik-objlib)
target_link_libraries(robot_localization
        robotik-objlib
        ${robot_model_msgs_LIBRARIES}
        ${PCL_LIBRARIES})

# todo: cleanup these deps
ament_target_dependencies(robot_localization
        "rclcpp"
        "diagnostic_msgs"
        "lifecycle_msgs"
        "sensor_msgs"
        "std_srvs"
        "robot_model_msgs"
        "rclcpp_components"
        "rclcpp_lifecycle")

install(TARGETS robot_localization
        EXPORT export_${PROJECT_NAME}
        #DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

