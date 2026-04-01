/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// rpy.hpp

#pragma once

#include <atomic>
#include <fins/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include "../ros_context.hpp"

class TransformRPY : public fins::Node {
public:
  void define() override {
    set_basics("TransformRPY", "Generates TransformStamped from RPY parameters at 50Hz.", "ROS>Transform");
    register_output<0, geometry_msgs::msg::TransformStamped>("transform");

    register_parameter<double>("tx", &TransformRPY::set_tx, 0.0);
    register_parameter<double>("ty", &TransformRPY::set_ty, 0.0);
    register_parameter<double>("tz", &TransformRPY::set_tz, 0.0);
    register_parameter<double>("roll", &TransformRPY::set_roll, 0.0);
    register_parameter<double>("pitch", &TransformRPY::set_pitch, 0.0);
    register_parameter<double>("yaw", &TransformRPY::set_yaw, 0.0);
    register_parameter<std::string>("from_frame", &TransformRPY::set_from_frame, "map");
    register_parameter<std::string>("to_frame", &TransformRPY::set_to_frame, "base_link");
  }

  void initialize() override {
    ROSContext::get_instance().init();
    is_running_ = false;
    node_ = rclcpp::Node::make_shared("transform_rpy_node");
  }

  void run() override {
    is_running_ = true;
    worker_ = std::thread(&TransformRPY::loop, this);
  }

  void pause() override {
    is_running_ = false;
    if (worker_.joinable())
      worker_.join();
  }

  void reset() override { pause(); }

  void set_tx(const double &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    tx_ = v;
  }
  void set_ty(const double &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    ty_ = v;
  }
  void set_tz(const double &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    tz_ = v;
  }
  void set_roll(const double &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    r_ = v;
  }
  void set_pitch(const double &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    p_ = v;
  }
  void set_yaw(const double &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    y_ = v;
  }
  void set_from_frame(const std::string &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    from_frame_ = v;
  }
  void set_to_frame(const std::string &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    to_frame_ = v;
  }

private:
  void loop() {
    while (is_running_) {
      geometry_msgs::msg::TransformStamped t;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        t.header.stamp = node_->now();
        t.header.frame_id = from_frame_;
        t.child_frame_id = to_frame_;

        tf2::Quaternion q;
        q.setRPY(r_ * M_PI / 180.0, p_ * M_PI / 180.0, y_ * M_PI / 180.0);

        t.transform.translation.x = tx_;
        t.transform.translation.y = ty_;
        t.transform.translation.z = tz_;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
      }
      send<0>(t, fins::now());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  std::mutex mutex_;
  double tx_ = 0, ty_ = 0, tz_ = 0;
  double r_ = 0, p_ = 0, y_ = 0;
  std::string from_frame_ = "map";
  std::string to_frame_ = "base_link";

  rclcpp::Node::SharedPtr node_;
  std::atomic<bool> is_running_{false};
  std::thread worker_;
};

EXPORT_NODE(TransformRPY)