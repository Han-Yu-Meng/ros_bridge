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
      shutting_down_.store(false);
      rclcpp::InitOptions init_options;
      
#if RCLCPP_VERSION_MAJOR < 8
      rclcpp::init(argc, argv, init_options);
#else
      rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);
#endif

      rclcpp::ExecutorOptions options;
      exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(options, 4);

      node_ = rclcpp::Node::make_shared("fins_ros_bridge");
      exec_->add_node(node_);

      spinning_ = true;
      spin_thread_ = std::thread([this]() {
        pthread_setname_np(pthread_self(), "ros_spin");
        exec_->spin();
      });
    }
  }

  bool io_ready() const {
    return rclcpp::ok() && !shutting_down_.load();
  }

  bool is_shutting_down() const {
    return shutting_down_.load();
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
    shutting_down_.store(true);

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

      if (exec_ && node_) {
        exec_->remove_node(node_);
      }
      node_.reset();
      exec_.reset();

      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }
  }

private:
  ROSContext() = default;
  ~ROSContext() {
    shutdown();
  }

  std::atomic<bool> spinning_{false};
  std::atomic<bool> shutting_down_{false};
  std::thread spin_thread_;
  rclcpp::Executor::SharedPtr exec_;
  rclcpp::Node::SharedPtr node_;
};