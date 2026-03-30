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
    if (ros_node_)
      ROSContext::get_instance().remove_node(ros_node_);
    sub_.reset();
    ros_node_.reset();
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

    if (ros_node_)
      ROSContext::get_instance().remove_node(ros_node_);

    std::string node_name = "sub_" + std::to_string((uint64_t)this);
    ros_node_ = rclcpp::Node::make_shared(node_name);

    auto cb = [this](const typename ROSMsgT::SharedPtr msg) {
      this->send_ptr<0>(msg, fins::now());
    };

    sub_ = ros_node_->create_subscription<ROSMsgT>(topic_, 10, cb);

    ROSContext::get_instance().add_node(ros_node_);
    logger->info("Subscribed to {}", topic_);
  }

  std::string topic_;
  rclcpp::Node::SharedPtr ros_node_;
  typename rclcpp::Subscription<ROSMsgT>::SharedPtr sub_;
};

#define DEFINE_ROS_SUB_NODE(ClassName, ROSMsgT, Desc)                          \
  class ClassName : public ROSSubNode<ROSMsgT> {                               \
  public:                                                                      \
    void define() override {                                                   \
      set_basics(#ClassName, Desc, "ROS>Subscriber");                          \
      register_output<0, ROSMsgT>("data");                                     \
      register_parameter<std::string>("ros_topic", &ClassName::set_topic,      \
                                      "/topic");                               \
    }                                                                          \
  };                                                                           \
  EXPORT_NODE(ClassName)

template <typename ROSMsgT> class ROSSensorSubNode : public fins::Node {
public:
  void define() override {}

  void initialize() override {
    ROSContext::get_instance().init();
    last_timestamp_ns_ = 0;
    total_msgs_received_ = 0;
    disorder_count_ = 0;
    create_subscriber();
  }

  void run() override {}
  void pause() override {}

  void reset() override {
    if (ros_node_)
      ROSContext::get_instance().remove_node(ros_node_);
    sub_.reset();
    callback_group_.reset();
    ros_node_.reset();
    last_timestamp_ns_ = 0;
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

    if (ros_node_)
      ROSContext::get_instance().remove_node(ros_node_);

    std::string node_name = "sub_sensor_" + std::to_string((uint64_t)this);
    ros_node_ = rclcpp::Node::make_shared(node_name);

    callback_group_ = ros_node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    auto qos = rclcpp::QoS(100);

    auto cb = [this](const typename ROSMsgT::SharedPtr msg) {
      int64_t curr_ns = rclcpp::Time(msg->header.stamp).nanoseconds();

      total_msgs_received_++;

      if (last_timestamp_ns_ > 0) {
        if (curr_ns < last_timestamp_ns_) {
          disorder_count_++;
          double diff_ms = (last_timestamp_ns_ - curr_ns) / 1000000.0;

          logger->warn(
              "Topic [{}]: TIME BACKWARDS! Prev: {}, Curr: {}, Lag: {:.3f}ms",
              topic_, last_timestamp_ns_, curr_ns, diff_ms);
        }
      }

      last_timestamp_ns_ = curr_ns;

      this->send_ptr<0>(msg, fins::now());
    };

    sub_ =
        ros_node_->create_subscription<ROSMsgT>(topic_, qos, cb, sub_options);

    ROSContext::get_instance().add_node(ros_node_);
    logger->info("Subscribed to Sensor topic {} (Serialized with Debug)",
                 topic_);
  }

  std::string topic_;
  rclcpp::Node::SharedPtr ros_node_;
  typename rclcpp::Subscription<ROSMsgT>::SharedPtr sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  int64_t last_timestamp_ns_ = 0;
  uint64_t total_msgs_received_ = 0;
  uint64_t disorder_count_ = 0;
};

#define DEFINE_ROS_SENSOR_SUB_NODE(ClassName, ROSMsgT, Desc)                   \
  class ClassName : public ROSSensorSubNode<ROSMsgT> {                         \
  public:                                                                      \
    void define() override {                                                   \
      set_basics(#ClassName, Desc, "ROS>SensorSubscriber");                    \
      register_output<0, ROSMsgT>("data");                                     \
      register_parameter<std::string>("ros_topic", &ClassName::set_topic,      \
                                      "/topic");                               \
    }                                                                          \
  };                                                                           \
  EXPORT_NODE(ClassName)
