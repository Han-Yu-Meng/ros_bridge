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

      spinning_ = true;
      exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      spin_thread_ = std::thread([this]() {
        while (spinning_ && rclcpp::ok()) {
          exec_->spin();
        }
      });
    }
  }

  void add_node(rclcpp::Node::SharedPtr node) {
    if (exec_)
      exec_->add_node(node);
  }

  void remove_node(rclcpp::Node::SharedPtr node) {
    if (exec_)
      exec_->remove_node(node);
  }

private:
  ROSContext() = default;
  ~ROSContext() {
    if (spinning_) {
      spinning_ = false;
      if (exec_)
        exec_->cancel();
      if (spin_thread_.joinable())
        spin_thread_.join();
      rclcpp::shutdown();
    }
  }

  bool spinning_ = false;
  std::thread spin_thread_;
  rclcpp::Executor::SharedPtr exec_;
};