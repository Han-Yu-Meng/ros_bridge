/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// ros_sub.hpp

#pragma once

#include <atomic>
#include <fins/node.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "ros_context.hpp"

template <typename ROSMsgT> class ROSSubNode : public fins::Node {
public:
  void define() override {}

  void initialize() override {
    ROSContext::get_instance().init();
    paused_.store(false);
    create_subscriber();
  }

  void run() override {
    if (paused_.exchange(false)) {
      create_subscriber();
    }
  }

  void pause() override {
    paused_.store(true);
    std::lock_guard<std::mutex> lock(state_mutex_);
    sub_.reset();
  }

  void reset() override {
    paused_.store(true);
    std::lock_guard<std::mutex> lock(state_mutex_);
    sub_.reset();
    topic_.clear();
  }

  void set_topic(const std::string &topic) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (topic_ != topic) {
      topic_ = topic;
      if (!paused_.load()) {
        create_subscriber_locked();
      }
    }
  }

protected:
  void create_subscriber() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    create_subscriber_locked();
  }

  void create_subscriber_locked() {
    if (topic_.empty() || paused_.load() || !ROSContext::get_instance().io_ready())
      return;

    auto node = ROSContext::get_instance().get_node();
    if (!node) return;

    auto cb = [this](const typename ROSMsgT::SharedPtr msg) {
      if (!paused_.load()) {
        this->send_ptr("msg", msg, fins::now());
      }
    };

    sub_ = node->create_subscription<ROSMsgT>(topic_, 10, cb);

    logger->info("Subscribed to {}", topic_);
  }

  std::string topic_;
  std::atomic<bool> paused_{false};
  std::mutex state_mutex_;
  typename rclcpp::Subscription<ROSMsgT>::SharedPtr sub_;
};

#define DEFINE_ROS_SUB_NODE(ClassName, ROSMsgT, Desc)                          \
  class ClassName : public ROSSubNode<ROSMsgT> {                               \
  public:                                                                      \
    void define() override {                                                   \
      set_basics(#ClassName, Desc, "ROS>Subscriber");                          \
      register_output<ROSMsgT>("msg");                                     \
      register_parameter<std::string>("topic", &ClassName::set_topic,      \
                                      "/topic");                               \
    }                                                                          \
  };                                                                           \
  EXPORT_NODE(ClassName)
