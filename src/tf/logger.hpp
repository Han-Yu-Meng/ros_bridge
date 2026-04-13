/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// logger.hpp

#pragma once

#include <fins/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "../ros_context.hpp"

class TFLogger : public fins::Node {
public:
  TFLogger() = default;

  void define() override {
    set_name("TFLogger");
    set_description("Logs TF transform messages using built-in logger.");
    set_category("ROS>Transform");

    register_input<geometry_msgs::msg::TransformStamped>("transform", &TFLogger::on_transform);
  }

  void initialize() override { logger->info("TFLogger initialized."); }

  void run() override {}
  void pause() override {}
  void reset() override {}

  void on_transform(const geometry_msgs::msg::TransformStamped &tf) {
    const auto &trans = tf.transform.translation;
    const auto &rot = tf.transform.rotation;

    logger->info("TF Transform | Time: {}.{} | Frame: {} -> {} | T: [{:.3f}, {:.3f}, {:.3f}] | R: [{:.3f}, {:.3f}, "
                 "{:.3f}, {:.3f}]",
                 tf.header.stamp.sec, tf.header.stamp.nanosec, tf.header.frame_id, tf.child_frame_id, trans.x, trans.y,
                 trans.z, rot.x, rot.y, rot.z, rot.w);
  }
};

EXPORT_NODE(TFLogger)
