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
    livox_ros_driver2
    cv_bridge
    pcl_conversions
)

fins_add_node(${PROJECT_NAME} src/nodes.cpp)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
fins_link_ros_dependencies(${PROJECT_NAME} ${ROS_REQUIRED_PACKAGES})

foreach(_PKG ${ROS_OPTIONAL_PACKAGES})
    fins_optional_ros_dependency(${PROJECT_NAME} ${_PKG})
endforeach()