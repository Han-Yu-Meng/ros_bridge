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
    paused_.store(true);
  }

  void run() override {
    if (paused_.exchange(false)) {
      create_subscriber();
    }
  }

  void pause() override {
    paused_.store(true);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (ROSContext::get_instance().io_ready()) {
      sub_.reset();
    }
  }

  void reset() override {
    paused_.store(true);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (ROSContext::get_instance().io_ready()) {
      sub_.reset();
    }
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

  void set_history(const std::string &history) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (history_ != history) {
      history_ = history;
      if (!paused_.load()) {
        create_subscriber_locked();
      }
    }
  }

  void set_depth(int depth) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (depth_ != depth) {
      depth_ = depth;
      if (!paused_.load()) {
        create_subscriber_locked();
      }
    }
  }

  void set_reliability(const std::string &reliability) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (reliability_ != reliability) {
      reliability_ = reliability;
      if (!paused_.load()) {
        create_subscriber_locked();
      }
    }
  }

  void set_durability(const std::string &durability) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (durability_ != durability) {
      durability_ = durability;
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

    rclcpp::QoS qos(depth_);
    if (history_ == "Keep All") {
      qos.keep_all();
    } else {
      qos.keep_last(depth_);
    }

    if (reliability_ == "Best Effort") {
      qos.best_effort();
    } else {
      qos.reliable();
    }

    if (durability_ == "Transient Local") {
      qos.transient_local();
    } else {
      qos.durability_volatile();
    }

    // Disable default QoS event callbacks to prevent SIGSEGV when
    // the subscriber is destroyed while the executor processes
    // incompatible QoS events for it.
    rclcpp::SubscriptionOptions options;
    options.use_default_callbacks = false;

    sub_ = node->create_subscription<ROSMsgT>(topic_, qos, cb, options);

    logger->info("Subscribed to {} (QoS: history={}, depth={}, reliability={}, durability={})",
                 topic_, history_, depth_, reliability_, durability_);
  }

  std::string topic_;
  std::string history_{"Keep Last"};
  int depth_{10};
  std::string reliability_{"Reliable"};
  std::string durability_{"Volatile"};
  std::atomic<bool> paused_{false};
  std::mutex state_mutex_;
  typename rclcpp::Subscription<ROSMsgT>::SharedPtr sub_;
};

#define DEFINE_ROS_SUB_NODE(ClassName, ROSMsgT, Desc)                          \
  class ClassName : public ROSSubNode<ROSMsgT> {                               \
  public:                                                                      \
    void define() override {                                                   \
      set_name(#ClassName);                                                    \
      set_description(Desc);                                                   \
      set_category("ROS>Subscriber");                                          \
      register_output<ROSMsgT>("msg");                                         \
      register_parameter<std::string>("topic", &ClassName::set_topic,          \
                                      "/topic");                               \
      register_parameter<std::string>("history", &ClassName::set_history,      \
                                      "Keep Last");                            \
      register_parameter<int>("depth", &ClassName::set_depth, 10);             \
      register_parameter<std::string>("reliability",                           \
                                      &ClassName::set_reliability, "Reliable"); \
      register_parameter<std::string>("durability",                            \
                                      &ClassName::set_durability, "Volatile");  \
    }                                                                          \
  };                                                                           \
  EXPORT_NODE(ClassName)
