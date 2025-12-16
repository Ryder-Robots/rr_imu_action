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

#include "rr_imu_action/rr_imu_action_node.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;
using MonitorImuAction = rr_interfaces::action::MonitorImuAction;

namespace rr_imu_action
{
    CallbackReturn RrImuActionNode::on_configure(const State &state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "configuring %s", get_name());
        declare_parameter("transport_plugin", "rrobots::interfaces::RRImuActionPluginIface");
        std::string plugin_param = get_parameter("transport_plugin").as_string();
        RCLCPP_DEBUG(get_logger(), "transport plugin is '%s'", plugin_param.c_str());
        CallbackReturn rv = CallbackReturn::SUCCESS;
        try {

            poly_loader_ = std::make_unique<pluginlib::ClassLoader<RRImuActionPluginIface>>("rr_common_base", "rrobots::interfaces::RRImuActionPluginIface");
            transport_ = poly_loader_->createUniqueInstance(plugin_param);
            rv = transport_->on_configure(state, shared_from_this());
        }
        catch (pluginlib::PluginlibException &ex) {
            RCLCPP_FATAL(get_logger(), "could not load transport plugin: %s - reported: %s", plugin_param.c_str(), ex.what());
            return CallbackReturn::ERROR;
        }

        return rv;
    }

    CallbackReturn RrImuActionNode::on_activate(const State &state)
    {
        RCLCPP_INFO(get_logger(), "activating %s", get_name());
        if (!transport_) {
            RCLCPP_ERROR(get_logger(), "Transport plugin not configured");
            return CallbackReturn::ERROR;
        }
        auto handle_goal = std::bind(&RRImuActionPluginIface::handle_goal, transport_.get(), std::placeholders::_1, std::placeholders::_2);
        auto handle_cancel = std::bind(&RRImuActionPluginIface::handle_cancel, transport_.get(), std::placeholders::_1);
        auto handle_accepted = std::bind(&RRImuActionPluginIface::handle_accepted, transport_.get(), std::placeholders::_1);

        action_server_ = rclcpp_action::create_server<MonitorImuAction>(
            this,
            "imu_monitor_action",
            handle_goal,
            handle_cancel,
            handle_accepted);

        return transport_->on_activate(state);
    }

    CallbackReturn RrImuActionNode::on_deactivate(const State &state)
    {
        RCLCPP_INFO(get_logger(), "deactivating %s", get_name());
        if (!transport_) {
            RCLCPP_WARN(get_logger(), "Transport plugin not configured");
            // return success, on non-configured plugin can still be considered
            // deactivated.
            return CallbackReturn::SUCCESS;
        }
        return transport_->on_deactivate(state);
    }

    CallbackReturn RrImuActionNode::on_cleanup(const State &state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "cleaning up %s", get_name());
        return CallbackReturn::SUCCESS;
    }

} // namespace rr_imu_action

RCLCPP_COMPONENTS_REGISTER_NODE(rr_imu_action::RrImuActionNode);
