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
/// \brief Declaration of the Nav2Bridge class.

#ifndef EASYNAV_NAV2_BRIDGE__NAV2BRIDGE_HPP_
#define EASYNAV_NAV2_BRIDGE__NAV2BRIDGE_HPP_

#include <cstdint>
#include <memory>
#include <type_traits>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "easynav_system/GoalManagerClient.hpp"

namespace easynav
{

/// nav2_msgs added error codes to NavigateToPose's result in Jazzy (Humble's is empty)
template<class ResultT, class = void>
struct has_error_code : std::false_type {};

template<class ResultT>
struct has_error_code<ResultT, std::void_t<decltype(std::declval<ResultT &>().error_code)>>
  : std::true_type {};

/// Sets @p result's error_code where the field exists (no-op on Humble)
template<class ResultT>
void set_error_code(ResultT & result, uint16_t code)
{
  if constexpr (has_error_code<ResultT>::value) {
    result.error_code = code;
  } else {
    (void)result;
    (void)code;
  }
}

/**
 * @class Nav2Bridge
 * @brief Exposes a nav2_msgs/action/NavigateToPose action server backed by EasyNav.
 *
 * Every accepted action goal is forwarded to EasyNav through a GoalManagerClient,
 * and EasyNav's feedback/result are translated back into NavigateToPose feedback
 * and result, so that a Nav2-compatible client cannot tell EasyNav from Nav2.
 *
 * Follows the same single-threaded, timer-driven pattern used by other EasyNav
 * GoalManagerClient users (see PatrollingNode): a periodic cycle() call polls
 * the client's state instead of blocking a dedicated thread per goal.
 */
class Nav2Bridge : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Nav2Bridge)

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit Nav2Bridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// @brief Lazily creates gm_client_ and the action server (needs shared_from_this()).
  void initialize();

  /// @brief Periodic callback: polls EasyNav's state and drives the current goal.
  void cycle();

  /// @brief Decide whether to accept an incoming NavigateToPose goal.
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);

  /// @brief Decide whether to accept a cancellation request.
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  /// @brief Take ownership of an accepted goal: preempts any goal in progress
  /// and forwards the new one to EasyNav. cycle() drives it from here on.
  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  bool initialized_ {false};

  /// @brief The NavigateToPose action server.
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

  /// @brief Client used to forward goals to EasyNav.
  GoalManagerClient::SharedPtr gm_client_;

  /// @brief The goal handle currently owning EasyNav navigation, if any.
  std::shared_ptr<GoalHandleNavigateToPose> current_goal_handle_;

  /// @brief Drives cycle() periodically.
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace easynav

#endif  // EASYNAV_NAV2_BRIDGE__NAV2BRIDGE_HPP_
