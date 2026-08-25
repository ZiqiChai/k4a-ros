// Copyright 2026 Ziqi CHAI
// SPDX-License-Identifier: BSD-3-Clause

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "k4a_ros/k4a_driver_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<k4a_ros::K4ADriverNode>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("k4a_driver"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
