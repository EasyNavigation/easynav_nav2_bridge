// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "easynav_nav2_bridge/Nav2Bridge.hpp"
#include "easynav_system/GoalManager.hpp"
#include "easynav_common/types/NavState.hpp"

#include "action_msgs/srv/cancel_goal.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"

using namespace std::chrono_literals;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace
{

// Mirrors Nav2Bridge's own kUnknownErrorCode: not every nav2_msgs build
// declares a named UNKNOWN error code (some message-only distributions only
// define NONE=0), so tests check the bridge's actual numeric contract instead.
constexpr uint16_t kUnknownErrorCode = 9000;
constexpr uint16_t kNoErrorCode = 0;

// Humble's NavigateToPose result has no error_code: there only the result code is checked
template<class ResultT>
void expect_error_code(const ResultT & result, uint16_t expected)
{
  if constexpr (easynav::has_error_code<ResultT>::value) {
    EXPECT_EQ(result.error_code, expected);
  } else {
    (void)result;
    (void)expected;
  }
}

/// @brief Collects everything a NavigateToPose action client observes for one goal.
struct GoalTracking
{
  std::vector<NavigateToPose::Feedback> feedbacks;
  bool result_received {false};
  rclcpp_action::ResultCode result_code {rclcpp_action::ResultCode::UNKNOWN};
  NavigateToPose::Result::SharedPtr result;
};

geometry_msgs::msg::PoseStamped
make_goal_pose(double x)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.orientation.w = 1.0;
  return pose;
}

}  // namespace

class Nav2BridgeTestCase : public ::testing::Test
{
protected:
  // rclcpp::init()/shutdown() run exactly once for the whole suite rather
  // than once per test: repeatedly tearing down and recreating the context
  // in the same process, with a NavigateToPose action server/client having
  // existed in it, reliably hangs the *next* rclcpp::init() (reproduced both
  // locally and in CI) — action server/client DDS entities apparently don't
  // tolerate that cycle as cleanly as plain pub/sub does. Nodes/clients are
  // still fully created and destroyed per test in SetUp()/TearDown() below.
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    // Must be created here rather than as a plain member: as a plain member it would be
    // default-constructed together with the test fixture itself, i.e. before
    // SetUp() (and therefore before rclcpp::init()) ever runs.
    exe = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    nav_state = std::make_shared<easynav::NavState>();
    nav_state->set("robot_pose", nav_msgs::msg::Odometry());

    // Namespaced under the test's own name so every topic/service each test's
    // nodes create (rosout, the control topic, the action's 5 DDS entities...)
    // is fully distinct from every other test's: reusing the same names
    // across tests in the same process, right after the previous test tore
    // its own nodes down, reliably hangs the *next* test's node/action-server
    // creation (reproduced both locally and in CI) -- the DDS entities from
    // the previous, same-named node/action apparently don't finish going away
    // as fast as the C++ objects that owned them do.
    const std::string ns =
      std::string("/") + ::testing::UnitTest::GetInstance()->current_test_info()->name();

    client_node = rclcpp::Node::make_shared("nav2_client_node", ns);
    bridge = easynav::Nav2Bridge::make_shared(
      rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=" + ns}));
    system_node = rclcpp_lifecycle::LifecycleNode::make_shared("system_node", ns);

    exe->add_node(client_node);
    exe->add_node(bridge);
    exe->add_node(system_node->get_node_base_interface());

    gm_server = easynav::GoalManager::make_shared(*nav_state, system_node);

    action_client = rclcpp_action::create_client<NavigateToPose>(client_node, "navigate_to_pose");

    auto start = client_node->now();
    while (!action_client->action_server_is_ready() && client_node->now() - start < 2s) {
      spin_for(10ms);
    }
    ASSERT_TRUE(action_client->action_server_is_ready());
  }

  void TearDown() override
  {
    // Explicitly release every node/client/executor between tests, ahead of
    // the fixture's implicit member teardown, so the next test starts from a
    // clean graph rather than relying on destruction order.
    action_client.reset();
    exe.reset();
    bridge.reset();
    gm_server.reset();
    system_node.reset();
    client_node.reset();
  }

  /// @brief Spins the executor and advances the (simulated) EasyNav backend.
  void spin_for(std::chrono::milliseconds duration)
  {
    rclcpp::Rate rate(50);
    auto start = client_node->now();
    while (client_node->now() - start < rclcpp::Duration(duration)) {
      gm_server->update(*nav_state);
      exe->spin_some();
      rate.sleep();
    }
  }

  /// @brief Sends a goal and blocks (spinning) until it is accepted or rejected.
  GoalHandle::SharedPtr send_goal_and_wait_accepted(
    double x, const std::shared_ptr<GoalTracking> & tracking)
  {
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = make_goal_pose(x);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.feedback_callback =
      [tracking](
      GoalHandle::SharedPtr /*handle*/,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        tracking->feedbacks.push_back(*feedback);
      };
    options.result_callback =
      [tracking](const GoalHandle::WrappedResult & result) {
        tracking->result_received = true;
        tracking->result_code = result.code;
        tracking->result = result.result;
      };

    auto goal_handle_future = action_client->async_send_goal(goal_msg, options);

    auto start = client_node->now();
    while (goal_handle_future.wait_for(0s) != std::future_status::ready &&
      client_node->now() - start < 2s)
    {
      gm_server->update(*nav_state);
      exe->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return goal_handle_future.get();
  }

  /// @brief Blocks (spinning) until a result has been received for the given goal.
  void wait_for_result(
    const std::shared_ptr<GoalTracking> & tracking,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
  {
    auto start = client_node->now();
    while (!tracking->result_received && client_node->now() - start < rclcpp::Duration(timeout)) {
      gm_server->update(*nav_state);
      exe->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::shared_ptr<easynav::NavState> nav_state;
  rclcpp::Node::SharedPtr client_node;
  rclcpp_lifecycle::LifecycleNode::SharedPtr system_node;

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe;

  easynav::Nav2Bridge::SharedPtr bridge;
  easynav::GoalManager::SharedPtr gm_server;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client;
};

TEST_F(Nav2BridgeTestCase, normal_navigation_succeeds)
{
  auto tracking = std::make_shared<GoalTracking>();
  auto goal_handle = send_goal_and_wait_accepted(5.0, tracking);
  ASSERT_NE(goal_handle, nullptr);

  spin_for(300ms);

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  ASSERT_FALSE(tracking->feedbacks.empty());

  gm_server->set_finished();
  wait_for_result(tracking);

  ASSERT_TRUE(tracking->result_received);
  ASSERT_EQ(tracking->result_code, rclcpp_action::ResultCode::SUCCEEDED);
  expect_error_code(*tracking->result, kNoErrorCode);
  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
}

TEST_F(Nav2BridgeTestCase, navigation_failure_is_reported_as_aborted)
{
  auto tracking = std::make_shared<GoalTracking>();
  auto goal_handle = send_goal_and_wait_accepted(5.0, tracking);
  ASSERT_NE(goal_handle, nullptr);

  spin_for(300ms);
  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);

  gm_server->set_failed("obstacle blocking path");
  wait_for_result(tracking);

  ASSERT_TRUE(tracking->result_received);
  ASSERT_EQ(tracking->result_code, rclcpp_action::ResultCode::ABORTED);
  expect_error_code(*tracking->result, kUnknownErrorCode);
}

TEST_F(Nav2BridgeTestCase, navigation_error_is_reported_as_aborted)
{
  auto tracking = std::make_shared<GoalTracking>();
  auto goal_handle = send_goal_and_wait_accepted(5.0, tracking);
  ASSERT_NE(goal_handle, nullptr);

  spin_for(300ms);
  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);

  gm_server->set_error("internal error");
  wait_for_result(tracking);

  ASSERT_TRUE(tracking->result_received);
  ASSERT_EQ(tracking->result_code, rclcpp_action::ResultCode::ABORTED);
  expect_error_code(*tracking->result, kUnknownErrorCode);
}

TEST_F(Nav2BridgeTestCase, client_requested_cancel_is_forwarded_to_easynav)
{
  auto tracking = std::make_shared<GoalTracking>();
  auto goal_handle = send_goal_and_wait_accepted(5.0, tracking);
  ASSERT_NE(goal_handle, nullptr);

  spin_for(300ms);
  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);

  auto cancel_future = action_client->async_cancel_goal(goal_handle);
  auto start = client_node->now();
  while (cancel_future.wait_for(0s) != std::future_status::ready &&
    client_node->now() - start < 2s)
  {
    gm_server->update(*nav_state);
    exe->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(cancel_future.wait_for(0s), std::future_status::ready);
  ASSERT_EQ(cancel_future.get()->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);

  wait_for_result(tracking);

  ASSERT_TRUE(tracking->result_received);
  ASSERT_EQ(tracking->result_code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
}

TEST_F(Nav2BridgeTestCase, new_goal_preempts_previous_one)
{
  auto tracking1 = std::make_shared<GoalTracking>();
  auto goal_handle1 = send_goal_and_wait_accepted(5.0, tracking1);
  ASSERT_NE(goal_handle1, nullptr);

  spin_for(300ms);
  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);

  auto tracking2 = std::make_shared<GoalTracking>();
  auto goal_handle2 = send_goal_and_wait_accepted(8.0, tracking2);
  ASSERT_NE(goal_handle2, nullptr);

  spin_for(300ms);

  // The first goal must have been terminated by the preemption.
  wait_for_result(tracking1, 1000ms);
  ASSERT_TRUE(tracking1->result_received);
  ASSERT_EQ(tracking1->result_code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_FALSE(tracking2->result_received);

  // EasyNav must now be navigating towards the second goal.
  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  const auto goals = gm_server->get_goals();
  ASSERT_EQ(goals.goals.size(), 1u);
  ASSERT_DOUBLE_EQ(goals.goals[0].pose.position.x, 8.0);

  gm_server->set_finished();
  wait_for_result(tracking2);

  ASSERT_TRUE(tracking2->result_received);
  ASSERT_EQ(tracking2->result_code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(Nav2BridgeTestCase, external_preemption_is_reported_as_aborted_not_canceled)
{
  auto tracking = std::make_shared<GoalTracking>();
  auto goal_handle = send_goal_and_wait_accepted(5.0, tracking);
  ASSERT_NE(goal_handle, nullptr);

  spin_for(300ms);
  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);

  // Some other client (e.g. RViz's "2D Nav Goal") takes over navigation directly,
  // bypassing this bridge and its GoalManagerClient entirely.
  auto pose_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10);
  spin_for(100ms);
  pose_pub->publish(make_goal_pose(9.0));

  wait_for_result(tracking, 1000ms);

  ASSERT_TRUE(tracking->result_received);
  ASSERT_EQ(tracking->result_code, rclcpp_action::ResultCode::ABORTED);
  expect_error_code(*tracking->result, kUnknownErrorCode);
}

TEST_F(Nav2BridgeTestCase, sequential_goals_after_success_are_independent)
{
  for (int i = 0; i < 3; ++i) {
    auto tracking = std::make_shared<GoalTracking>();
    auto goal_handle = send_goal_and_wait_accepted(5.0 + i, tracking);
    ASSERT_NE(goal_handle, nullptr) << "iteration " << i;

    spin_for(300ms);
    ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE) << "iteration " << i;

    gm_server->set_finished();
    wait_for_result(tracking);

    ASSERT_TRUE(tracking->result_received) << "iteration " << i;
    ASSERT_EQ(tracking->result_code, rclcpp_action::ResultCode::SUCCEEDED) << "iteration " << i;
    ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE) << "iteration " << i;
  }
}

TEST_F(Nav2BridgeTestCase, sequential_goals_after_failure_are_independent)
{
  auto tracking1 = std::make_shared<GoalTracking>();
  auto goal_handle1 = send_goal_and_wait_accepted(5.0, tracking1);
  ASSERT_NE(goal_handle1, nullptr);
  spin_for(300ms);

  gm_server->set_failed("Reason 1");
  wait_for_result(tracking1);
  ASSERT_EQ(tracking1->result_code, rclcpp_action::ResultCode::ABORTED);

  auto tracking2 = std::make_shared<GoalTracking>();
  auto goal_handle2 = send_goal_and_wait_accepted(6.0, tracking2);
  ASSERT_NE(goal_handle2, nullptr);
  spin_for(300ms);
  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);

  gm_server->set_finished();
  wait_for_result(tracking2);
  ASSERT_EQ(tracking2->result_code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(Nav2BridgeTestCase, feedback_is_translated_from_easynav)
{
  auto tracking = std::make_shared<GoalTracking>();
  auto goal_handle = send_goal_and_wait_accepted(5.0, tracking);
  ASSERT_NE(goal_handle, nullptr);

  spin_for(300ms);

  ASSERT_FALSE(tracking->feedbacks.empty());
  const auto & feedback = tracking->feedbacks.back();

  const auto last_easynav_feedback = gm_server->get_goals();
  ASSERT_EQ(last_easynav_feedback.goals.size(), 1u);
  ASSERT_DOUBLE_EQ(feedback.current_pose.pose.position.x, 0.0);
  ASSERT_GE(feedback.distance_remaining, 0.0f);

  gm_server->set_finished();
  wait_for_result(tracking);
}
