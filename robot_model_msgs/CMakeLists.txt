cmake_minimum_required(VERSION 3.5)
project(robot_model_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/CompliantJointParams.msg"
        "msg/JointCalibration.msg"
        "msg/Contact.msg"
        "msg/SupportState.msg"
        "msg/ControlState.msg"
        "msg/EffectorState.msg"
        "msg/ExternalForce.msg"
        "msg/Forces.msg"
        "msg/Limb.msg"
        "msg/ModelState.msg"
        "msg/MultiSegmentTrajectory.msg"
        "msg/SegmentTrajectory.msg"
        "msg/TrajectoryProgress.msg"
        "msg/MultiTrajectoryProgress.msg"
        "msg/TrajectoryGoal.msg"
        "msg/TrajectoryEvent.msg"
        "msg/TrajectoryComplete.msg"
        "action/EffectorTrajectory.action"
        DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        )

ament_export_dependencies(rosidl_default_runtime)
#ament_export_include_directories(include)

ament_package()
