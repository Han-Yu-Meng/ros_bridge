/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// ros_context.hpp

#pragma once
#include <atomic>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

class ROSContext {
public:
  static ROSContext &get_instance() {
    static ROSContext instance;
    return instance;
  }

  void init(int argc = 0, char **argv = nullptr) {
    if (!rclcpp::ok()) {
      rclcpp::init(argc, argv);

      exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      node_ = rclcpp::Node::make_shared("fins_ros_bridge");
      exec_->add_node(node_);

      spinning_ = true;
      spin_thread_ = std::thread([this]() {
        exec_->spin();
      });
    }
  }

  rclcpp::Node::SharedPtr get_node() {
    return node_;
  }

  void add_node(rclcpp::Node::SharedPtr node) {
    if (exec_ && node != node_)
      exec_->add_node(node);
  }

  void remove_node(rclcpp::Node::SharedPtr node) {
    if (exec_ && node != node_)
      exec_->remove_node(node);
  }

  void shutdown() {
    bool expected_spinning = true;
    if (spinning_.compare_exchange_strong(expected_spinning, false)) {
      if (exec_) {
        exec_->cancel();
      }
      if (spin_thread_.joinable()) {
        if (std::this_thread::get_id() != spin_thread_.get_id()) {
          spin_thread_.join();
        } else {
          spin_thread_.detach();
        }
      }
      rclcpp::shutdown();
    }
  }

private:
  ROSContext() = default;
  ~ROSContext() {
    shutdown();
  }

  std::atomic<bool> spinning_{false};
  std::thread spin_thread_;
  rclcpp::Executor::SharedPtr exec_;
  rclcpp::Node::SharedPtr node_;
};