cmake_minimum_required(VERSION 3.16)

project(ros_bridge)

set(ROS_REQUIRED_PACKAGES
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2_ros
    tf2_geometry_msgs
)

set(ROS_OPTIONAL_PACKAGES
    cv_bridge
    pcl_conversions
)

fins_add_node(${PROJECT_NAME} src/nodes.cpp)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
fins_link_ros_dependencies(${PROJECT_NAME} ${ROS_REQUIRED_PACKAGES})

foreach(_PKG ${ROS_OPTIONAL_PACKAGES})
    fins_optional_ros_dependency(${PROJECT_NAME} ${_PKG})
endforeach()

find_package(livox_ros_driver2 QUIET)
if(livox_ros_driver2_FOUND)
    message(STATUS ">> Optional Package 'livox_ros_driver2': FOUND. Enabling...")
    target_compile_definitions(${PROJECT_NAME} PRIVATE WITH_LIVOX_ROS_DRIVER2)
    if(TARGET livox_ros_driver2::livox_interfaces2__rosidl_generator_cpp)
        set(LIVOX_MSG_PREFIX livox_interfaces2)
        target_compile_definitions(${PROJECT_NAME} PRIVATE LIVOX_INTERFACES2)
    else()
        set(LIVOX_MSG_PREFIX livox_ros_driver2)
    endif()
    target_link_libraries(${PROJECT_NAME} PRIVATE
        livox_ros_driver2::${LIVOX_MSG_PREFIX}__rosidl_generator_cpp
        livox_ros_driver2::${LIVOX_MSG_PREFIX}__rosidl_typesupport_cpp
    )
else()
    message(STATUS ">> Optional Package 'livox_ros_driver2': NOT FOUND. Skipping.")
endif()