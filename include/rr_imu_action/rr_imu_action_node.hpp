// Copyright (c) 2025 Ryder Robots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "rr_imu_action/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rr_common_base/rr_imu_action_plugin_iface.hpp"
#include <pluginlib/class_loader.hpp>

namespace rr_imu_action
{

  class RrImuActionNode
  {
  private:
    pluginlib::ClassLoader<rrobots::interfaces::RRImuActionPluginIface> poly_loader_;

  public:
    explicit RrImuActionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : rclcpp_lifecycle::LifecycleNode("rr_imu_action_node", options),
          poly_loader_("rr_common_base", "rrobots::interfaces::RrNodeJoyPluginIface")
    {
    }

    virtual ~RrImuActionNode() = default;
  };

} // namespace rr_imu_action
