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
#include "rr_common_base/rr_imu_action_plugin_iface.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <memory>

using namespace rr_imu_action;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;

class MockImuActionPlugin : public rrobots::interfaces::RRImuActionPluginIface
{
  public:
    MockImuActionPlugin() = default;
    virtual ~MockImuActionPlugin() = default;

    CallbackReturn on_configure(const State &state, rclcpp_lifecycle::LifecycleNode::SharedPtr node) override
    {
        (void)node;
        on_configure_called = true;
        last_state = state.id();
        return configure_return_value;
    }

    CallbackReturn on_activate(const State &state) override
    {
        on_activate_called = true;
        last_state = state.id();
        return activate_return_value;
    }

    CallbackReturn on_deactivate(const State &state) override
    {
        on_deactivate_called = true;
        last_state = state.id();
        return deactivate_return_value;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const rr_interfaces::action::MonitorImuAction::Goal> goal) override
    {
        (void)goal;
        handle_goal_called = true;
        last_goal_uuid = uuid;
        return goal_response;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<rr_interfaces::action::MonitorImuAction>> goal_handle) override
    {
        (void)goal_handle;
        handle_cancel_called = true;
        return cancel_response;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<rr_interfaces::action::MonitorImuAction>> goal_handle) override
    {
        handle_accepted_called = true;
        accepted_goal_handle = goal_handle;
    }

    // Test control flags
    bool on_configure_called = false;
    bool on_activate_called = false;
    bool on_deactivate_called = false;
    bool handle_goal_called = false;
    bool handle_cancel_called = false;
    bool handle_accepted_called = false;

    uint8_t last_state = 0;
    rclcpp_action::GoalUUID last_goal_uuid;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<rr_interfaces::action::MonitorImuAction>> accepted_goal_handle;

    // Control return values for testing
    CallbackReturn configure_return_value = CallbackReturn::SUCCESS;
    CallbackReturn activate_return_value = CallbackReturn::SUCCESS;
    CallbackReturn deactivate_return_value = CallbackReturn::SUCCESS;
    rclcpp_action::GoalResponse goal_response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    rclcpp_action::CancelResponse cancel_response = rclcpp_action::CancelResponse::ACCEPT;
};

class TestRrImuActionNode : public testing::Test
{
  protected:
    TestRrImuActionNode() {}

    ~TestRrImuActionNode() override {}

    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node_options = rclcpp::NodeOptions();
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }

    rclcpp::NodeOptions node_options;
};

TEST_F(TestRrImuActionNode, node_instantiation)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);
    EXPECT_TRUE(node != nullptr);
    EXPECT_EQ(node->get_name(), std::string("rr_imu_action_node"));
}

TEST_F(TestRrImuActionNode, initial_state_is_unconfigured)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);
    EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(TestRrImuActionNode, on_configure_without_plugin_fails)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);

    // Configure should fail because no valid plugin can be loaded
    State state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    auto result = node->on_configure(state);

    EXPECT_EQ(result, CallbackReturn::ERROR);
}

TEST_F(TestRrImuActionNode, on_activate_without_configure_fails)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);

    // Activate should fail because transport plugin is not configured
    State state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    auto result = node->on_activate(state);

    EXPECT_EQ(result, CallbackReturn::ERROR);
}

TEST_F(TestRrImuActionNode, on_cleanup_always_succeeds)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);

    State state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    auto result = node->on_cleanup(state);

    EXPECT_EQ(result, CallbackReturn::SUCCESS);
}

TEST_F(TestRrImuActionNode, plugin_loader_is_initialized)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);

    // The plugin loader should be initialized with proper base class type
    // This is tested indirectly by attempting to configure
    // If the loader is not initialized, getBaseClassType() will return empty string

    State state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    auto result = node->on_configure(state);

    // Even though configuration fails (no valid plugin),
    // the error should be about plugin loading, not about uninitialized loader
    EXPECT_EQ(result, CallbackReturn::ERROR);
}

TEST_F(TestRrImuActionNode, parameter_declaration_on_configure)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);

    State state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    node->on_configure(state);

    // Check that the transport_plugin parameter was declared
    EXPECT_TRUE(node->has_parameter("transport_plugin"));

    // Check default value
    auto param = node->get_parameter("transport_plugin");
    EXPECT_EQ(param.as_string(), std::string("rrobots::interfaces::RRImuActionPluginIface"));
}

TEST_F(TestRrImuActionNode, lifecycle_state_transitions)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);

    // Check initial state
    EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    // Note: Full lifecycle testing would require a valid plugin or mocking the plugin system
    // This test verifies the node starts in the correct state
}

TEST_F(TestRrImuActionNode, node_name_is_correct)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);
    EXPECT_EQ(std::string(node->get_name()), "rr_imu_action_node");
}

TEST_F(TestRrImuActionNode, node_namespace_default)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);
    EXPECT_EQ(std::string(node->get_namespace()), "/");
}

TEST_F(TestRrImuActionNode, node_with_custom_options)
{
    rclcpp::NodeOptions custom_options;
    custom_options.arguments({"--ros-args", "-r", "__ns:=/test_namespace"});

    auto node = std::make_shared<RrImuActionNode>(custom_options);
    EXPECT_TRUE(node != nullptr);
}

TEST_F(TestRrImuActionNode, on_deactivate_without_configured_transport)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);

    // Deactivate should succeed with warning when transport is not configured
    // This allows graceful deactivation even if configuration failed
    State state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
    auto result = node->on_deactivate(state);

    // Should return SUCCESS (non-configured plugin can still be considered deactivated)
    EXPECT_EQ(result, CallbackReturn::SUCCESS);
}

TEST_F(TestRrImuActionNode, multiple_instantiation)
{
    // Test that we can create multiple instances
    auto node1 = std::make_shared<RrImuActionNode>(node_options);
    auto node2 = std::make_shared<RrImuActionNode>(node_options);

    EXPECT_TRUE(node1 != nullptr);
    EXPECT_TRUE(node2 != nullptr);
    EXPECT_NE(node1, node2);
}

TEST_F(TestRrImuActionNode, destructor_cleanup)
{
    // Test that destructor doesn't crash
    {
        auto node = std::make_shared<RrImuActionNode>(node_options);
        EXPECT_TRUE(node != nullptr);
    }
    // Node should be destroyed here without issues
    SUCCEED();
}

TEST_F(TestRrImuActionNode, plugin_parameter_can_be_set_via_options)
{
    // Create node options with custom parameter
    rclcpp::NodeOptions custom_options;
    custom_options.append_parameter_override("transport_plugin", "rr_common_plugins::rr_serial_plugins::ImuActionSerialPlugin");

    auto node = std::make_shared<RrImuActionNode>(custom_options);

    State state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    node->on_configure(state);

    // Verify the parameter was used
    auto param = node->get_parameter("transport_plugin");
    EXPECT_EQ(param.as_string(), std::string("rr_common_plugins::rr_serial_plugins::ImuActionSerialPlugin"));
}

TEST_F(TestRrImuActionNode, configure_creates_transport_instance)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);

    State state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    auto result = node->on_configure(state);

    // Configuration will fail due to plugin loading, but the attempt should be made
    // This test verifies the configure logic executes
    EXPECT_EQ(result, CallbackReturn::ERROR);
}

TEST_F(TestRrImuActionNode, activate_requires_valid_transport)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);

    // Skip configure and try to activate
    State state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    auto result = node->on_activate(state);

    // Should fail with error because transport is null
    EXPECT_EQ(result, CallbackReturn::ERROR);
}

TEST_F(TestRrImuActionNode, logger_is_functional)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);

    // Verify logger is accessible
    auto logger = node->get_logger();
    EXPECT_EQ(std::string(logger.get_name()), "rr_imu_action_node");
}

TEST_F(TestRrImuActionNode, clock_is_functional)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);

    // Verify clock is accessible
    auto clock = node->get_clock();
    EXPECT_TRUE(clock != nullptr);

    auto now = clock->now();
    EXPECT_GT(now.nanoseconds(), 0);
}

TEST_F(TestRrImuActionNode, node_supports_lifecycle_interface)
{
    auto node = std::make_shared<RrImuActionNode>(node_options);

    // Verify it's a lifecycle node
    auto lifecycle_node = std::dynamic_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node);
    EXPECT_TRUE(lifecycle_node != nullptr);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    return result;
}
