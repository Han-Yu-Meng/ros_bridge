/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// nodes.cpp

#include "ros_pub.hpp"
#include "ros_sub.hpp"

#include "tf/broadcaster.hpp"
#include "tf/static_broadcaster.hpp"
#include "tf/logger.hpp"
#include "tf/lookup.hpp"
#include "tf/rpy.hpp"
#include "tf/wxyz.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#ifdef WITH_LIVOX_ROS_DRIVER2
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

REGISTER_PLUGIN_INIT({ ROSContext::get_instance().init(); })
REGISTER_PLUGIN_DESTROY({ ROSContext::get_instance().shutdown(); })

/* Define ROS Publisher Nodes */

DEFINE_ROS_PUB_NODE(PointCloudPublisher, sensor_msgs::msg::PointCloud2, "Publishes PointCloud2")
DEFINE_ROS_PUB_NODE(OdometryPublisher, nav_msgs::msg::Odometry, "Publishes Odometry")
DEFINE_ROS_PUB_NODE(PathPublisher, nav_msgs::msg::Path, "Publishes Path")
DEFINE_ROS_PUB_NODE(PoseStampedPublisher, geometry_msgs::msg::PoseStamped, "Publishes PoseStamped")
DEFINE_ROS_PUB_NODE(MarkerArrayPublisher, visualization_msgs::msg::MarkerArray, "Publishes Markers")
DEFINE_ROS_PUB_NODE(TransformStampedPublisher, geometry_msgs::msg::TransformStamped, "Publishes TransformStamped")
DEFINE_ROS_PUB_NODE(TwistPublisher, geometry_msgs::msg::Twist, "Publishes Twist")
DEFINE_ROS_PUB_NODE(ImuPublisher, sensor_msgs::msg::Imu, "Publishes IMU")
DEFINE_ROS_PUB_NODE(OccupancyGridPublisher, nav_msgs::msg::OccupancyGrid, "Publishes OccupancyGrid")
DEFINE_ROS_PUB_NODE(ImagePublisher, sensor_msgs::msg::Image, "Publishes Image")
#ifdef WITH_LIVOX_ROS_DRIVER2
DEFINE_ROS_PUB_NODE(LivoxCustomMsgPublisher, livox_ros_driver2::msg::CustomMsg, "Publishes Livox CustomMsg")
#endif

/* Define ROS Subscriber Nodes */

DEFINE_ROS_SUB_NODE(PointCloudSubscriber, sensor_msgs::msg::PointCloud2, "Subscribes to PointCloud2")
DEFINE_ROS_SUB_NODE(ImuSubscriber, sensor_msgs::msg::Imu, "Subscribes to IMU")
DEFINE_ROS_SUB_NODE(OdometrySubscriber, nav_msgs::msg::Odometry, "Subscribes to Odometry")
DEFINE_ROS_SUB_NODE(PoseStampedSubscriber, geometry_msgs::msg::PoseStamped, "Subscribes to PoseStamped")
DEFINE_ROS_SUB_NODE(MarkerArraySubscriber, visualization_msgs::msg::MarkerArray, "Subscribes to MarkerArray")
DEFINE_ROS_SUB_NODE(TransformStampedSubscriber, geometry_msgs::msg::TransformStamped, "Subscribes to TransformStamped")
DEFINE_ROS_SUB_NODE(TwistSubscriber, geometry_msgs::msg::Twist, "Subscribes to Twist")
DEFINE_ROS_SUB_NODE(OccupancyGridSubscriber, nav_msgs::msg::OccupancyGrid, "Subscribes to OccupancyGrid")
DEFINE_ROS_SUB_NODE(ImageSubscriber, sensor_msgs::msg::Image, "Subscribes to Image")
#ifdef WITH_LIVOX_ROS_DRIVER2
DEFINE_ROS_SUB_NODE(LivoxCustomMsgSubscriber, livox_ros_driver2::msg::CustomMsg, "Subscribes to Livox CustomMsg")
#endif

DEFINE_PLUGIN_ENTRY(fins::STATELESS)