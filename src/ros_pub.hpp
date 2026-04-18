/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// ros_pub.hpp

#pragma once

#include <atomic>
#include <fins/node.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "ros_context.hpp"

template<typename ROSMsgT>
class ROSPubNode : public fins::Node {
public:
  void define() override {}

  void initialize() override {
    ROSContext::get_instance().init();
    paused_.store(false);
    create_publisher();
  }

  void run() override {
    if (paused_.exchange(false)) {
      create_publisher();
    }
  }

  void pause() override {
    paused_.store(true);
    std::lock_guard<std::mutex> lock(state_mutex_);
    pub_.reset();
  }

  void reset() override {
    paused_.store(true);
    std::lock_guard<std::mutex> lock(state_mutex_);
    pub_.reset();
    topic_.clear();
  }

  void receive_msg(const ROSMsgT &msg) {
    if (paused_.load() || !ROSContext::get_instance().io_ready())
      return;

    typename rclcpp::Publisher<ROSMsgT>::SharedPtr pub;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pub = pub_;
    }

    if (pub && ROSContext::get_instance().io_ready()) {
      pub->publish(msg);
    }
  }

  void set_topic(const std::string &topic) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (topic_ != topic) {
      topic_ = topic;
      if (!paused_.load()) {
        create_publisher_locked();
      }
    }
  }

protected:
  void create_publisher() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    create_publisher_locked();
  }

  void create_publisher_locked() {
    if (topic_.empty() || paused_.load() || !ROSContext::get_instance().io_ready())
      return;

    auto node = ROSContext::get_instance().get_node();
    if (!node) return;

    pub_ = node->create_publisher<ROSMsgT>(topic_, 10);
    logger->info("Publish to topic: {}", topic_);
  }

  std::string topic_;
  std::atomic<bool> paused_{false};
  std::mutex state_mutex_;
  typename rclcpp::Publisher<ROSMsgT>::SharedPtr pub_;
};

#define DEFINE_ROS_PUB_NODE(ClassName, ROSMsgT, Desc)                                \
  class ClassName : public ROSPubNode<ROSMsgT> {                                     \
  public:                                                                            \
    void define() override {                                                         \
      set_basics(#ClassName, Desc, "ROS>Publisher");                                 \
      register_input<ROSMsgT>("msg", &ClassName::receive_msg);                   \
      register_parameter<std::string>("topic", &ClassName::set_topic, "/topic"); \
    }                                                                                \
  };                                                                                 \
  EXPORT_NODE(ClassName)
