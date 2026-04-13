/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// lookup.hpp

#pragma once

#include <atomic>
#include <fins/node.hpp>
#include <mutex>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include "../ros_context.hpp"

class LookupTransform : public fins::Node {
public:
  void define() override {
    set_name("LookupTransform");
    set_description("Looks up TF transforms between frames at 50Hz.");
    set_category("ROS>Transform");

    register_output<geometry_msgs::msg::TransformStamped>("transform");

    register_parameter<std::string>("from_frame", &LookupTransform::set_from_frame, "map");
    register_parameter<std::string>("to_frame", &LookupTransform::set_to_frame, "base_link");

    register_parameter<int>("timeout_ms", &LookupTransform::set_timeout, 100);
    register_parameter<double>("frequency", &LookupTransform::set_frequency, 50.0);
  }

  void initialize() override {
    ROSContext::get_instance().init();
    is_running_ = false;

    auto node = ROSContext::get_instance().get_node();
    if (!node) {
      logger->error("Failed to get ROS node for transform lookup!");
      return;
    }

    buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node);
  }

  void run() override {
    is_running_ = true;
    worker_ = std::thread(&LookupTransform::loop, this);
  }

  void pause() override {
    is_running_ = false;
    if (worker_.joinable())
      worker_.join();
  }

  void reset() override { pause(); }

  void set_from_frame(const std::string &s) {
    std::lock_guard<std::mutex> lock(mutex_);
    from_frame_ = s;
  }
  void set_to_frame(const std::string &s) {
    std::lock_guard<std::mutex> lock(mutex_);
    to_frame_ = s;
  }
  void set_timeout(const int &i) {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_ms_ = i;
  }
  void set_frequency(const double &f) {
    std::lock_guard<std::mutex> lock(mutex_);
    frequency_ = f;
  }

private:
  void loop() {
    while (is_running_) {
      std::string from, to;
      int timeout;
      double freq;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        from = from_frame_;
        to = to_frame_;
        timeout = timeout_ms_;
        freq = frequency_;
      }

      if (!from.empty() && !to.empty()) {
        try {
          auto tf = buffer_->lookupTransform(from, to, tf2::TimePointZero, tf2::durationFromSec(timeout / 1000.0));
          send("transform", tf);
        } catch (const tf2::TransformException &e) {
          logger->warn("Failed to lookup transform from {} to {}: {}", from, to, e.what());
        }
      }

      if (freq > 0.0) {
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(1000000.0 / freq)));
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
  }

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;

  std::mutex mutex_;
  std::string from_frame_;
  std::string to_frame_;
  int timeout_ms_ = 100;
  double frequency_ = 50.0;

  std::thread worker_;
  std::atomic<bool> is_running_{false};
};

EXPORT_NODE(LookupTransform)