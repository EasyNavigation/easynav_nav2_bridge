# easynav_nav2_bridge

[![rolling](https://github.com/EasyNavigation/easynav_nav2_bridge/actions/workflows/rolling.yaml/badge.svg?branch=rolling)](https://github.com/EasyNavigation/easynav_nav2_bridge/actions/workflows/rolling.yaml)

A drop-in `nav2_msgs/action/NavigateToPose` action server backed by **EasyNav**.

It lets any Nav2-compatible client — RViz's "2D Nav Goal", the Nav2 simple
commander, a BehaviorTree `NavigateToPose` node, `ros2 action send_goal`, etc. —
drive EasyNav without knowing it isn't talking to Nav2's `bt_navigator`. The
bridge forwards the goal to EasyNav through an
[`easynav::GoalManagerClient`](https://github.com/EasyNavigation/EasyNavigation/blob/rolling/easynav_system/include/easynav_system/GoalManagerClient.hpp)
and translates EasyNav's feedback and result back into the Nav2 action's
feedback and result.

## Why

EasyNav has its own native goal protocol
(`easynav_interfaces/msg/NavigationControl`, driven through
`GoalManagerClient`/`GoalManager`). Some tools and integrations, however, only
know how to speak Nav2's action interface. This package is the adapter between
the two: on one side it is a `NavigateToPose` action server, on the other a
regular EasyNav client, so a client can't tell the two systems apart.

## Node

### `nav2_bridge_main`

Runs an `easynav::Nav2Bridge` node.

```bash
ros2 run easynav_nav2_bridge nav2_bridge_main
```

It advertises the action:

| Action | Type |
|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `feedback_rate` | double | `10.0` | Rate (Hz) at which EasyNav's state is polled and feedback is republished. |

The goal's `behavior_tree` field is ignored: EasyNav does not use behavior
trees, so there is nothing to select there.

## How it works

`Nav2Bridge` owns a single `GoalManagerClient` and a single `NavigateToPose`
action server, and drives both from one periodic, single-threaded `cycle()`
callback — the same timer-driven pattern used by other EasyNav clients (see
`easynav_patrolling_behavior`). No extra threads or locks are involved.

* **Accepting a goal**: every incoming goal is accepted
  (`ACCEPT_AND_EXECUTE`) and immediately forwarded to EasyNav via
  `GoalManagerClient::send_goal()`. EasyNav — not this bridge — is the one
  that actually decides whether a goal is achievable.
* **Feedback**: while EasyNav reports it is navigating, each `cycle()` tick
  republishes its latest feedback (`current_pose`, `navigation_time`,
  `estimated_time_remaining`, mapped to `distance_remaining`) as
  `NavigateToPose::Feedback`.
* **Success**: when EasyNav reports the goal as finished, the action
  succeeds with `error_code == NONE`.
* **Failure / internal error**: reported as `ABORTED`, with a non-`NONE`
  `error_code`. EasyNav's textual reason is logged server-side (`RCLCPP_ERROR`)
  rather than placed in the result, since not every `nav2_msgs` build declares
  an `error_msg` field (message-only distributions may trim `NavigateToPose`
  down to just `error_code`/`NONE`).
* **Client-requested cancel**: a Nav2 cancel request is forwarded to EasyNav
  (`GoalManagerClient::cancel()`); once EasyNav confirms the cancellation the
  action reports `CANCELED`.
* **Preemption by a new goal**: sending a new `NavigateToPose` goal while one
  is active preempts it, exactly like Nav2's `bt_navigator`. The previous
  goal is terminated as `ABORTED` ("Goal preempted by a new navigation
  request") and the new one takes over — under the hood this reuses
  EasyNav's own native preemption (`GoalManagerClient` transparently
  re-sends the goal), so there is no cancel/re-request round trip.
* **Preemption by someone else**: if a *different* EasyNav client (another
  `GoalManagerClient`, or a raw `goal_pose` publish, e.g. from RViz) takes
  over navigation while this bridge owns the active goal, the Nav2 goal is
  terminated as `ABORTED` rather than `CANCELED` — from the Nav2 client's
  point of view it never asked to cancel, so reporting `CANCELED` would be
  a protocol violation (and is in fact illegal at the `rclcpp_action` state
  machine level unless a cancel was actually requested).

## Building

This package depends on `easynav_system` (and the rest of the
[EasyNavigation](https://github.com/EasyNavigation/EasyNavigation) core
packages), plus `nav2_msgs`. In a workspace where those aren't already
available, checkout `EasyNavigation` as a sibling under `src/` (and, since
some distros don't ship `nav2_msgs` as a standalone package, the
[`nav2_msgs_only`](https://github.com/fmrico/navigation2/tree/nav2_msgs_only)
branch) before running `colcon build` — see
[`.github/thirdparty.repos`](.github/thirdparty.repos) for the exact sources
CI pulls in.

## Tests

`tests/nav2_bridge_tests.cpp` exercises the bridge end-to-end: a real
`nav2_msgs` action client talks to a `Nav2Bridge`, which in turn talks to a
real `easynav::GoalManager` standing in for the rest of EasyNav. Covered
scenarios include normal completion, failure, internal error, client-side
cancellation, preemption by a new Nav2 goal, preemption by another EasyNav
client, sequential goals, and feedback translation.

```bash
colcon test --packages-select easynav_nav2_bridge
```
