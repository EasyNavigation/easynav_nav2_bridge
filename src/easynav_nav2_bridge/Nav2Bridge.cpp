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

/// \file
/// \brief Implementation of the Nav2Bridge class.

#include <chrono>
#include <memory>
#include <string>

#include "easynav_nav2_bridge/Nav2Bridge.hpp"

namespace easynav
{

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace
{

// Numeric value of nav2_msgs' NavigateToPose::Result::UNKNOWN. Kept as a literal
// rather than referencing the named constant because not every nav2_msgs build
// declares it (e.g. message-only distributions trimmed down to just NONE=0);
// the field itself (error_code) is common to both, only the named error codes
// beyond NONE differ.
constexpr uint16_t kUnknownErrorCode = 9000;
// kNoErrorCode, also missing where the result has no error codes
constexpr uint16_t kNoErrorCode = 0;

/// @brief Convert EasyNav's latest feedback into a NavigateToPose feedback message.
Nav2Bridge::NavigateToPose::Feedback::SharedPtr to_feedback(
  const easynav_interfaces::msg::NavigationControl & control)
{
  auto feedback = std::make_shared<Nav2Bridge::NavigateToPose::Feedback>();
  feedback->current_pose = control.current_pose;
  feedback->navigation_time = control.navigation_time;
  feedback->estimated_time_remaining = control.estimated_time_remaining;
  feedback->distance_remaining = control.distance_to_goal;
  return feedback;
}

}  // namespace

Nav2Bridge::Nav2Bridge(const rclcpp::NodeOptions & options)
: Node("easynav_nav2_bridge_node", options)
{
  const double feedback_rate_hz = declare_parameter("feedback_rate", 10.0);
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / feedback_rate_hz));

  // Free function: Node::create_timer does not exist in Humble
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(period), std::bind(&Nav2Bridge::cycle, this));
}

void
Nav2Bridge::initialize()
{
  gm_client_ = GoalManagerClient::make_shared(shared_from_this());

  action_server_ = rclcpp_action::create_server<NavigateToPose>(
    shared_from_this(),
    "navigate_to_pose",
    std::bind(&Nav2Bridge::handle_goal, this, _1, _2),
    std::bind(&Nav2Bridge::handle_cancel, this, _1),
    std::bind(&Nav2Bridge::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse
Nav2Bridge::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const NavigateToPose::Goal>/*goal*/)
{
  // EasyNav decides acceptance itself (through GoalManager); every syntactically
  // valid request is accepted here and forwarded, exactly like Nav2 does.
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Nav2Bridge::handle_cancel(const std::shared_ptr<GoalHandleNavigateToPose>/*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
Nav2Bridge::handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  // A new goal preempts whatever this bridge is currently navigating towards.
  // GoalManagerClient::send_goal() below performs the actual EasyNav-side
  // preemption (SENT_PREEMPT), so the previous action goal just needs to be
  // terminated at the Nav2 action level.
  if (current_goal_handle_ && current_goal_handle_->is_active()) {
    RCLCPP_INFO(get_logger(), "Goal preempted by a new navigation request");
    auto preempted_result = std::make_shared<NavigateToPose::Result>();
    set_error_code(*preempted_result, kUnknownErrorCode);
    try {
      current_goal_handle_->abort(preempted_result);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Could not terminate preempted goal: %s", e.what());
    }
  }

  current_goal_handle_ = goal_handle;
  gm_client_->send_goal(goal_handle->get_goal()->pose);
}

void
Nav2Bridge::cycle()
{
  if (!initialized_) {
    initialize();
    initialized_ = true;
    return;
  }

  if (current_goal_handle_ == nullptr) {
    return;
  }

  if (!current_goal_handle_->is_active()) {
    // Reached a terminal state through some other path (e.g. it was preempted
    // and already terminated by a newer goal's handle_accepted()).
    current_goal_handle_.reset();
    return;
  }

  const bool canceling = current_goal_handle_->is_canceling();
  const auto gm_state = gm_client_->get_state();

  if (canceling && gm_state == GoalManagerClient::State::ACCEPTED_AND_NAVIGATING) {
    gm_client_->cancel();
  }

  switch (gm_state) {
    case GoalManagerClient::State::IDLE:
    case GoalManagerClient::State::SENT_GOAL:
      break;

    case GoalManagerClient::State::SENT_PREEMPT:
    case GoalManagerClient::State::ACCEPTED_AND_NAVIGATING:
      current_goal_handle_->publish_feedback(to_feedback(gm_client_->get_feedback()));
      break;

    case GoalManagerClient::State::NAVIGATION_FINISHED:
      {
        auto result = std::make_shared<NavigateToPose::Result>();
        set_error_code(*result, kNoErrorCode);
        // succeed() is not a legal transition once the goal is CANCELING.
        canceling ? current_goal_handle_->canceled(result) : current_goal_handle_->succeed(result);
        gm_client_->reset();
        current_goal_handle_.reset();
        break;
      }

    case GoalManagerClient::State::NAVIGATION_CANCELLED:
      {
        auto result = std::make_shared<NavigateToPose::Result>();
        if (canceling) {
          set_error_code(*result, kNoErrorCode);
          current_goal_handle_->canceled(result);
        } else {
          // Cancelled by someone other than this action client (e.g. another
          // GoalManagerClient): canceled() would be an illegal transition
          // here, since this goal handle was never put in CANCELING state.
          RCLCPP_WARN(
            get_logger(), "Navigation was cancelled by another client: %s",
            gm_client_->get_result().status_message.c_str());
          set_error_code(*result, kUnknownErrorCode);
          current_goal_handle_->abort(result);
        }
        gm_client_->reset();
        current_goal_handle_.reset();
        break;
      }

    case GoalManagerClient::State::NAVIGATION_FAILED:
    case GoalManagerClient::State::NAVIGATION_REJECTED:
    case GoalManagerClient::State::ERROR:
      {
        RCLCPP_ERROR(
          get_logger(), "Navigation ended with an error: %s",
          gm_client_->get_result().status_message.c_str());
        auto result = std::make_shared<NavigateToPose::Result>();
        set_error_code(*result, kUnknownErrorCode);
        canceling ? current_goal_handle_->canceled(result) : current_goal_handle_->abort(result);
        gm_client_->reset();
        current_goal_handle_.reset();
        break;
      }
  }
}

}  // namespace easynav
