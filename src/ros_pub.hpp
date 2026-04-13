/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// ros_pub.hpp

#pragma once

#include <fins/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "ros_context.hpp"

template<typename ROSMsgT>
class ROSPubNode : public fins::Node {
public:
  void define() override {}

  void initialize() override {
    ROSContext::get_instance().init();
    create_publisher();
  }

  void run() override {}
  void pause() override {}
  void reset() override {
    topic_ = "";
    pub_.reset();
  }

  void receive_msg(const ROSMsgT &msg) {
    if (pub_) {
      pub_->publish(msg);
    }
  }

  void set_topic(const std::string &topic) {
    if (topic_ != topic) {
      topic_ = topic;
      create_publisher();
    }
  }

protected:
  void create_publisher() {
    if (topic_.empty())
      return;

    auto node = ROSContext::get_instance().get_node();
    if (!node) return;

    pub_ = node->create_publisher<ROSMsgT>(topic_, 10);
    logger->info("Publish to topic: {}", topic_);
  }

  std::string topic_;
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
