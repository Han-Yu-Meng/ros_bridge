/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// ros_sub.hpp

#pragma once

#include <fins/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "ros_context.hpp"

template <typename ROSMsgT> class ROSSubNode : public fins::Node {
public:
  void define() override {}

  void initialize() override {
    ROSContext::get_instance().init();
    create_subscriber();
  }

  void run() override {}
  void pause() override {}

  void reset() override {
    sub_.reset();
    topic_ = "";
  }

  void set_topic(const std::string &topic) {
    if (topic_ != topic) {
      topic_ = topic;
      create_subscriber();
    }
  }

protected:
  void create_subscriber() {
    if (topic_.empty())
      return;

    auto node = ROSContext::get_instance().get_node();
    if (!node) return;

    auto cb = [this](const typename ROSMsgT::SharedPtr msg) {
      this->send_ptr("msg", msg, fins::now());
    };

    sub_ = node->create_subscription<ROSMsgT>(topic_, 10, cb);

    logger->info("Subscribed to {}", topic_);
  }

  std::string topic_;
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
