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
    paused_.store(true);
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

  void set_history(const std::string &history) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (history_ != history) {
      history_ = history;
      if (!paused_.load()) {
        create_publisher_locked();
      }
    }
  }

  void set_depth(int depth) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (depth_ != depth) {
      depth_ = depth;
      if (!paused_.load()) {
        create_publisher_locked();
      }
    }
  }

  void set_reliability(const std::string &reliability) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (reliability_ != reliability) {
      reliability_ = reliability;
      if (!paused_.load()) {
        create_publisher_locked();
      }
    }
  }

  void set_durability(const std::string &durability) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (durability_ != durability) {
      durability_ = durability;
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

    pub_.reset();
    pub_ = node->create_publisher<ROSMsgT>(topic_, qos);
    logger->info("Publish to topic: {} (QoS: history={}, depth={}, reliability={}, durability={})", 
                 topic_, history_, depth_, reliability_, durability_);
  }

  std::string topic_;
  std::string history_{"Keep Last"};
  int depth_{10};
  std::string reliability_{"Reliable"};
  std::string durability_{"Volatile"};
  std::atomic<bool> paused_{false};
  std::mutex state_mutex_;
  typename rclcpp::Publisher<ROSMsgT>::SharedPtr pub_;
};

#define DEFINE_ROS_PUB_NODE(ClassName, ROSMsgT, Desc)                                \
  class ClassName : public ROSPubNode<ROSMsgT> {                                     \
  public:                                                                            \
    void define() override {                                                         \
      set_basics(#ClassName, Desc, "ROS>Publisher");                                 \
      register_input<ROSMsgT>("msg", &ClassName::receive_msg);                       \
      register_parameter<std::string>("topic", &ClassName::set_topic, "/topic");     \
      register_parameter<std::string>("history", &ClassName::set_history,            \
                                      "Keep Last");                                  \
      register_parameter<int>("depth", &ClassName::set_depth, 10);                   \
      register_parameter<std::string>("reliability",                                 \
                                      &ClassName::set_reliability, "Reliable");      \
      register_parameter<std::string>("durability",                                  \
                                      &ClassName::set_durability, "Volatile");       \
    }                                                                                \
  };                                                                                 \
  EXPORT_NODE(ClassName)
