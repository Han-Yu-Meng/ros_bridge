/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// broadcaster.hpp

#pragma once

#include <fins/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <atomic>
#include <mutex>
#include <sstream>
#include <tf2_ros/transform_broadcaster.h>
#include "../ros_context.hpp"

class TFBroadcaster : public fins::Node {
public:
  TFBroadcaster() = default;

  void define() override {
    set_basics("TFBroadcaster", "Broadcasts TF transforms to ROS2.", "ROS>Transform");

    register_input<geometry_msgs::msg::TransformStamped>("transform", &TFBroadcaster::on_transform);

    register_parameter<std::string>("from_frame_override", &TFBroadcaster::set_from_frame, "NONE");
    register_parameter<std::string>("to_frame_override", &TFBroadcaster::set_to_frame, "NONE");
  }

  void initialize() override {
    ROSContext::get_instance().init();
    paused_.store(false);

    auto node = ROSContext::get_instance().get_node();
    if (!node) {
      logger->error("Failed to get ROS node!");
      return;
    }

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    logger->info("TFBroadcaster initialized with frames: {} -> {}", from_frame_, to_frame_);
  }

  void run() override {}
  void pause() override {
    paused_.store(true);
    std::lock_guard<std::mutex> lock(mutex_);
    broadcaster_.reset();
  }

  void reset() override {
    pause();
    from_frame_ = "NONE";
    to_frame_ = "NONE";
  }

  void on_transform(const geometry_msgs::msg::TransformStamped &msg) {
    if (paused_.load() || !ROSContext::get_instance().io_ready())
      return;

    geometry_msgs::msg::TransformStamped transform = msg;
    tf2_ros::TransformBroadcaster *broadcaster = nullptr;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!broadcaster_)
        return;

      if (!from_frame_.empty() && from_frame_ != "NONE")
        transform.header.frame_id = from_frame_;
      if (!to_frame_.empty() && to_frame_ != "NONE")
        transform.child_frame_id = to_frame_;

      broadcaster = broadcaster_.get();
    }

    if (broadcaster && ROSContext::get_instance().io_ready()) {
      broadcaster->sendTransform(transform);
    }
  }

  void set_from_frame(const std::string &frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    from_frame_ = frame;
  }

  void set_to_frame(const std::string &frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    to_frame_ = frame;
  }

private:
  std::string from_frame_ = "NONE";
  std::string to_frame_ = "NONE";
  std::atomic<bool> paused_{false};
  std::mutex mutex_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

EXPORT_NODE(TFBroadcaster)