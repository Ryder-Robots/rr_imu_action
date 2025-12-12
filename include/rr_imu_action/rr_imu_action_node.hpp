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

#include <memory>
#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rr_common_base/rr_imu_action_plugin_iface.hpp"
#include "rr_imu_action/visibility_control.h"
#include "rr_interfaces/action/monitor_imu_action.hpp"

namespace rr_imu_action
{

    /**
     * @class RrImuActionNode
     * @brief provides action interface to IMU
     * 
     * Concrete implementation of IMU actions is provided by plugin, this service hides the plumbing of
     * the implementation.
     */
    class RR_IMU_ACTION_PUBLIC RrImuActionNode : public rclcpp_lifecycle::LifecycleNode
    {
        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
        using State = rclcpp_lifecycle::State;
        using RRImuActionPluginIface = rrobots::interfaces::RRImuActionPluginIface;
        using MonitorImuAction = rr_interfaces::action::MonitorImuAction;

      public:
        explicit RrImuActionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : rclcpp_lifecycle::LifecycleNode("rr_imu_action_node", options),
              poly_loader_("", "")
        {
        }

        virtual ~RrImuActionNode() = default;

        /**
         * @fn on_configure
         * @brief ROS2 lifecycle on_configure hook.
         * @param state - The current lifecycle state
         * 
         * Loads plugin using ros2 parameter 'imu_action_plugin' if specified, otherwise uses default.
         * Calls the plugin_lib_->on_configure method to perform any initialization required for the plugin.
         * 
         * @return CallbackReturn::SUCCESS if plugin loaded and configured successfully
         *         CallbackReturn::ERROR if plugin loading or configuration fails
         */
        CallbackReturn on_configure(const State &state) override;

        /**
         * @fn on_activate
         * @brief ROS2 lifecycle hook on_activate
         * @param state - The current lifecycle state
         * 
         * makes action server available.
         * registers functions plugin_lib_->handle_goal(), plugin_lib_->handle_cancel, 
         * and plugin_lib_->handle_accepted() to action server.
         * @return CallbackReturn::SUCCESS if operation successful
         *         CallbackReturn::ERROR if recoverable error occurs
         *         CallbackReturn::FAILURE if unrecoverable error occurs
         */
        CallbackReturn on_activate(const State &state) override;

        /**
         * @fn on_deactivate
         * @brief ROS2 lifecycle hook on_deactivate
         * @param state - The current lifecycle state
         * 
         * included for completeness currently unimplemented.
         * 
         * @return always returns CallbackReturn::SUCCESS
         */
        CallbackReturn on_deactivate(const State &state) override;

        /**
         * @fn on_cleanup
         * @brief ROS2 lifecycle hook on_cleanup
         * @param state - The current lifecycle state
         * 
         * included for completeness currently unimplemented.
         * 
         * @return always returns CallbackReturn::SUCCESS
         */
        CallbackReturn on_cleanup(const State &state) override;

      private:
        pluginlib::ClassLoader<RRImuActionPluginIface> poly_loader_;
        rclcpp_action::Server<MonitorImuAction>::SharedPtr action_server_;
        std::shared_ptr<RRImuActionPluginIface> plugin_lib_;
    };

} // namespace rr_imu_action
