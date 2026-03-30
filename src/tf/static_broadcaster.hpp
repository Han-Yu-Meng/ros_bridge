/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// static_broadcaster.hpp

#pragma once

#include <fins/node.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "../ros_context.hpp"

class StaticBroadcaster : public fins::Node {
public:
  void define() override {
    set_name("StaticBroadcaster");
    set_description("Broadcasts a static TF transform once when received.");
    set_category("ROS>Transform");

    register_input<geometry_msgs::msg::TransformStamped>("transform", &StaticBroadcaster::on_receive);
  }

  void initialize() override {
    is_published_ = false;
    std::stringstream ss;
    ss << "finevision_static_tf_broadcaster_" << static_cast<const void *>(this) << "_"
       << std::chrono::steady_clock::now().time_since_epoch().count();
    node_ = rclcpp::Node::make_shared(ss.str());
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  }

  void run() override {}

  void pause() override {}

  void reset() override {
    is_published_ = false;
  }

  void on_receive(const fins::Msg<geometry_msgs::msg::TransformStamped> &msg) {
    if (!is_published_) {
      if (broadcaster_) {
        broadcaster_->sendTransform(*msg);
        is_published_ = true;
      }
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  bool is_published_ = false;
};

EXPORT_NODE(StaticBroadcaster)
