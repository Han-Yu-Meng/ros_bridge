/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// wxyz.hpp

#pragma once

#include <atomic>
#include <fins/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <thread>
#include "../ros_context.hpp"

class TransformWXYZ : public fins::Node {
public:
  void define() override {
    set_basics("TransformWXYZ", "Generates TransformStamped from Quaternion parameters at 50Hz.", "ROS>Transform");
    register_output<geometry_msgs::msg::TransformStamped>("transform");

    register_parameter<double>("tx", &TransformWXYZ::set_tx, 0.0);
    register_parameter<double>("ty", &TransformWXYZ::set_ty, 0.0);
    register_parameter<double>("tz", &TransformWXYZ::set_tz, 0.0);
    register_parameter<double>("qx", &TransformWXYZ::set_qx, 0.0);
    register_parameter<double>("qy", &TransformWXYZ::set_qy, 0.0);
    register_parameter<double>("qz", &TransformWXYZ::set_qz, 0.0);
    register_parameter<double>("qw", &TransformWXYZ::set_qw, 1.0);
    register_parameter<std::string>("from_frame", &TransformWXYZ::set_from_frame, "map");
    register_parameter<std::string>("to_frame", &TransformWXYZ::set_to_frame, "odom");
  }

  void initialize() override {
    ROSContext::get_instance().init();
    is_running_ = false;
  }

  void run() override {
    is_running_ = true;
    worker_ = std::thread(&TransformWXYZ::loop, this);
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
  void set_qx(const double &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    qx_ = v;
  }
  void set_qy(const double &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    qy_ = v;
  }
  void set_qz(const double &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    qz_ = v;
  }
  void set_qw(const double &v) {
    std::lock_guard<std::mutex> lock(mutex_);
    qw_ = v;
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
        auto node = ROSContext::get_instance().get_node();
        std::lock_guard<std::mutex> lock(mutex_);
        if (node) {
          t.header.stamp = node->now();
        } else {
          t.header.stamp = rclcpp::Clock().now();
        }
        t.header.frame_id = from_frame_;
        t.child_frame_id = to_frame_;

        t.transform.translation.x = tx_;
        t.transform.translation.y = ty_;
        t.transform.translation.z = tz_;
        t.transform.rotation.x = qx_;
        t.transform.rotation.y = qy_;
        t.transform.rotation.z = qz_;
        t.transform.rotation.w = qw_;
      }
      send("transform", t);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  std::mutex mutex_;
  double tx_ = 0, ty_ = 0, tz_ = 0;
  double qx_ = 0, qy_ = 0, qz_ = 0, qw_ = 1.0;
  std::string from_frame_ = "map";
  std::string to_frame_ = "odom";

  std::atomic<bool> is_running_{false};
  std::thread worker_;
};

EXPORT_NODE(TransformWXYZ)