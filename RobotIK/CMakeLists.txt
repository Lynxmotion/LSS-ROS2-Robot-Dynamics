
set(INCLUDE_FILES
        types.h model.h limb.h frame-ref.h
        contact.h state.h indexes.h effector.h
        conversions.h exception.h
        friction.h tween.h
        kinematics.h
        listeners/JointStateListener.h
        listeners/ModelStateListener.h
        listeners/URDFModelListener.h
        listeners/SegmentStateListener.h
        publishers/LifecycleManager.h
        publishers/JointControlPublisher.h)
set(SOURCE_FILES
        types.cpp model.cpp limb.cpp frame-ref.cpp
        contact.cpp state.cpp indexes.cpp effector.cpp
        conversions.cpp friction.cpp tween.cpp
        kinematics.cpp
        listeners/JointStateListener.cpp
        listeners/ModelStateListener.cpp
        listeners/URDFModelListener.cpp
        listeners/SegmentStateListener.cpp
        publishers/LifecycleManager.cpp
        publishers/JointControlPublisher.cpp)

list(TRANSFORM INCLUDE_FILES PREPEND include/)
list(TRANSFORM SOURCE_FILES PREPEND src/)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(srdfdom REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(PCL REQUIRED common geometry)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_model_msgs REQUIRED)

# we first add our intermediate object library
# which will them be linked into an archive and a shared library
#add_library(sigint-objlib OBJECT ${SOURCE_FILES} $<TARGET_OBJECTS:sigint-matrix> $<TARGET_OBJECTS:sigint-streams> $<TARGET_OBJECTS:sigint-numerical>)
add_library(robotik-objlib OBJECT ${INCLUDE_FILES} ${SOURCE_FILES})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})

# robotik dependancies
target_link_libraries(robotik-objlib pcl_common)
ament_target_dependencies(robotik-objlib
        "rclcpp"
        "rclcpp_lifecycle"
        "urdf"
        "urdfdom_headers"
        "srdfdom"
        "tf2"
        "tf2_kdl"
        "tf2_ros"
        "sensor_msgs"
        "robot_model_msgs"
        "visualization_msgs"
        "interactive_markers"
        "orocos_kdl"
        "kdl_parser")

# set compilation properties on the object library
set_property(TARGET robotik-objlib PROPERTY POSITION_INDEPENDENT_CODE 1)
target_compile_features(robotik-objlib PRIVATE cxx_std_14)
#set_property(TARGET robotik-objlib PROPERTY C_STANDARD 99)

# now link the object lib into static archive and shared library versions
set(ROBOTIK_TARGET_OBJECTS $<TARGET_OBJECTS:robotik-objlib> CACHE_PATH "RobotIK object files")
#add_library(robotik-static STATIC ${LSSLIB_TARGET_OBJECTS})
#add_library(robotik-shared SHARED ${LSSLIB_TARGET_OBJECTS})

#SET_TARGET_PROPERTIES(robotik-static PROPERTIES OUTPUT_NAME lss CLEAN_DIRECT_OUTPUT 1)
#SET_TARGET_PROPERTIES(robotik-shared PROPERTIES OUTPUT_NAME lss CLEAN_DIRECT_OUTPUT 1)
#target_link_libraries(robotik-shared ...)

# store location of sigint variables in the cache for other projects to reference
set(ROBOTIK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE PATH "path to RobotIK library includes")
set(ROBOTIK_LIBRARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "path to RobotIK library binaries")

#add_subdirectory(tests)

# attach version number to SO shared file
#set_target_properties(sigint-shared PROPERTIES
#        VERSION ${sigintSDK_VERSION_STRING}
#        SOVERSION ${sigintSDK_VERSION_MAJOR})

#install(TARGETS robotik-static robotik-shared
#        RUNTIME DESTINATION "bin"
#        LIBRARY DESTINATION "lib"
#        ARCHIVE DESTINATION "lib")
