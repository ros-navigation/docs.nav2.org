.. _navigate_through_poses:


Navigate Through Poses
**********************

This tutorial is for new users who want to:

1. Send a ``NavigateThroughPoses`` action goal.
2. Understand which ``NavigateThroughPosesNavigator`` functions are used at runtime.

Prerequisites
=============

- Ubuntu 24.04, ROS 2 Jazzy, Kilted or later, and Nav2 is installed.
- The ``nav2_minimal_turtlebot_simulation`` `repository <https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation>`_
  is included in the ROS 2 system install, no need to build or source it separately.
- Understand the ``navigate_through_poses_w_replanning_and_recovery.xml`` `behavior tree <https://docs.nav2.org/behavior_trees/trees/nav_through_poses_recovery.html>`_.
- Familiar with ROS 2 actions and `NavigateThroughPoses.action <https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/action/NavigateThroughPoses.action>`_ definition.

Start TB4 simulation
====================

Terminal 1:

.. code-block:: bash

   ros2 launch nav2_minimal_tb4_sim simulation.launch.py

This uses ``nav2_minimal_tb4_sim/launch/simulation.launch.py`` and spawns TB4 in Gazebo.

Start Nav2 stack
================

Terminal 2:

.. code-block:: bash

   ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True slam:=True autostart:=True

Notes:

- This repository provides simulation + robot model + BT navigator source.
- Full navigation servers (planner, controller, recoveries, lifecycle) come from Nav2.
- ``slam:=True`` is used so ``slam_launch.py`` starts and provides map creation + localization in a
  fresh simulation. With ``slam:=False``, Nav2 runs ``localization_launch.py`` and expects
  ``map:=...`` to be provided.

If you already have a map, prefer:

.. code-block:: bash

   ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True slam:=False map:=/path/to/map.yaml autostart:=True

- ``autostart:=True`` brings Nav2 lifecycle nodes up automatically so the
  ``navigate_through_poses`` action server is available. See also
  `example_nav_through_poses.py <https://github.com/ros-navigation/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_nav_through_poses.py>`_.

Send a goal
===========

Method 1: Using terminal command line
`````````````````````````````````````

Terminal 3:

.. code-block:: bash

   source ~/tb4_sim_ws/install/setup.bash
   ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
   '{
     poses: {
       header: {frame_id: map},
       goals: [
         {header: {frame_id: map}, pose: {position: {x: 3.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
         {header: {frame_id: map}, pose: {position: {x: 5.0, y: 1.0, z: 0.0}, orientation: {w: 0.25}}},
         {header: {frame_id: map}, pose: {position: {x: 7.0, y: 2.0, z: 0.0}, orientation: {w: 0.0}}}
       ]
     },
     behavior_tree: ""
   }' --feedback

``behavior_tree: ""`` means use the default ``default_nav_through_poses_bt_xml``
(default: ``navigate_through_poses_w_replanning_and_recovery.xml``).

Method 2: Using C++ API
```````````````````````
This method sends the exact same payload as Method 1:
three poses in ``map`` frame and ``behavior_tree: ""``.
Below is a minimal buildable package example.

Create a package:

.. code-block:: bash

   cd ~/tb4_sim_ws/src
   ros2 pkg create nav_through_poses_cpp_client --build-type ament_cmake \
     --dependencies rclcpp rclcpp_action nav2_msgs nav_msgs geometry_msgs

So ``nav_through_poses_cpp_client/package.xml`` should include the following dependencies:

.. code-block::

   <depend>rclcpp</depend>
   <depend>rclcpp_action</depend>
   <depend>nav2_msgs</depend>
   <depend>nav_msgs</depend>
   <depend>geometry_msgs</depend>

Update ``nav_through_poses_cpp_client/CMakeLists.txt``:

.. code-block:: cmake

   find_package(rclcpp REQUIRED)
   find_package(rclcpp_action REQUIRED)
   find_package(nav2_msgs REQUIRED)
   find_package(nav_msgs REQUIRED)
   find_package(geometry_msgs REQUIRED)

   add_executable(
     navigate_through_poses_client
     src/navigate_through_poses_client.cpp
   )

   ament_target_dependencies(
     navigate_through_poses_client
     rclcpp
     rclcpp_action
     nav2_msgs
     nav_msgs
     geometry_msgs
   )

   install(
     TARGETS navigate_through_poses_client
     DESTINATION lib/${PROJECT_NAME}
   )

Create ``nav_through_poses_cpp_client/src/navigate_through_poses_client.cpp``:

.. code-block:: cpp

    #include <chrono>
    #include <memory>
    #include <string>
    #include <utility>
    #include <vector>

    #include "geometry_msgs/msg/pose_stamped.hpp"
    #include "nav2_msgs/action/navigate_through_poses.hpp"
    #include "rclcpp/rclcpp.hpp"
    #include "rclcpp_action/rclcpp_action.hpp"


    class NavigateThroughPosesClient : public rclcpp::Node
    {
    public:
      using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
      using GoalHandleNavigateThroughPoses =
        rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

      NavigateThroughPosesClient()
      : Node("navigate_through_poses_client")
      {
        client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(
          this, "/navigate_through_poses");
      }

      void send_goal()
      {
        using namespace std::chrono_literals;

        if (!client_ptr_->wait_for_action_server(10s)) {
          RCLCPP_ERROR(get_logger(), "Action server /navigate_through_poses not available");
          rclcpp::shutdown();
          return;
        }

        auto goal_msg = NavigateThroughPoses::Goal();
        goal_msg.poses.header.frame_id = "map";
        goal_msg.poses.header.stamp = now();
        goal_msg.poses.goals = {
          make_pose(3.5, 0.0, 1.0),
          make_pose(5.0, 1.0, 0.25),
          make_pose(7.0, 2.0, 0.0)
        };
        goal_msg.behavior_tree = "";

        auto send_goal_options =
          rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();

        send_goal_options.goal_response_callback =
          [this](const GoalHandleNavigateThroughPoses::SharedPtr & goal_handle) {
            if (!goal_handle) {
              RCLCPP_ERROR(get_logger(), "Goal rejected by action server");
              rclcpp::shutdown();
              return;
            }
            RCLCPP_INFO(get_logger(), "Goal accepted");
          };

        send_goal_options.feedback_callback =
          [this](
          GoalHandleNavigateThroughPoses::SharedPtr,
          const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
            RCLCPP_INFO_THROTTLE(
              this->get_logger(),
              *this->get_clock(),
              500,  // 500ms
              "Feedback: distance_remaining=%.2f, poses_remaining=%d",
              feedback->distance_remaining,
              static_cast<int>(feedback->number_of_poses_remaining));
          };

        send_goal_options.result_callback =
          [this](const GoalHandleNavigateThroughPoses::WrappedResult & result) {
            switch (result.code) {
              case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "Navigation succeeded");
                break;
              case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(
                  get_logger(), "Navigation aborted: code=%u msg=%s",
                  result.result->error_code, result.result->error_msg.c_str());
                break;
              case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(get_logger(), "Navigation canceled");
                break;
              default:
                RCLCPP_ERROR(get_logger(), "Unknown result code");
                break;
            }
            rclcpp::shutdown();
          };

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
      }

    private:
      geometry_msgs::msg::PoseStamped make_pose(double x, double y, double w)
      {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = w;
        return pose;
      }

      rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;
    };

    int main(int argc, char ** argv)
    {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<NavigateThroughPosesClient>();
      node->send_goal();
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
    }


Build and run:

.. code-block:: bash

   cd ~/tb4_sim_ws
   colcon build --packages-select nav_through_poses_cpp_client
   source install/setup.bash
   ros2 run nav_through_poses_cpp_client navigate_through_poses_client

Method 3: Using Python API (Simple Commander)
`````````````````````````````````````````````
This method uses ``nav2_simple_commander`` and sends ``nav_msgs/Goals`` to
``BasicNavigator.goThroughPoses(...)``.

Version note (as of February 22, 2026):

- Upstream ``navigation2`` branch heads differ in
  ``nav2_simple_commander/nav2_simple_commander/robot_navigator.py``:
  ``jazzy`` and ``kilted`` still show
  ``self.info(f'Navigating with {len(goal_msg.poses)} goals....')``,
  while Rolling development (``main``) shows
  ``self.info(f'Navigating with {len(poses.goals)} goals....')``.
- If your installed package still has ``len(goal_msg.poses)``, you can
  temporarily comment out that log line or change it to ``len(poses.goals)``.
- Some binary installs or local edits may already include this fix. Verify your
  local ``robot_navigator.py`` before editing.

Create ``~/tb4_sim_ws/go_through_poses.py``:

.. code-block:: python

   #!/usr/bin/env python3

   import time

   import rclpy
   from geometry_msgs.msg import PoseStamped
   from nav_msgs.msg import Goals
   from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


   def make_pose(navigator: BasicNavigator, x: float, y: float, w: float = 1.0) -> PoseStamped:
       pose = PoseStamped()
       pose.header.frame_id = "map"
       pose.header.stamp = navigator.get_clock().now().to_msg()
       pose.pose.position.x = x
       pose.pose.position.y = y
       pose.pose.orientation.w = w
       return pose


   def main() -> None:
       rclpy.init()
       navigator = BasicNavigator(node_name="navigate_through_poses_py_client")

       # If you launched Nav2 with slam:=True, use localizer='robot_localization'.
       # For AMCL localization mode, use waitUntilNav2Active() with defaults.
       navigator.waitUntilNav2Active(localizer="robot_localization")

       goals_msg = Goals()
       goals_msg.header.frame_id = "map"
       goals_msg.header.stamp = navigator.get_clock().now().to_msg()
       goals_msg.goals = [
           make_pose(navigator, 3.5, 0.0, 1.0),
           make_pose(navigator, 5.0, 1.0, 0.25),
           make_pose(navigator, 7.0, 2.0, 0.0),
       ]

       task = navigator.goThroughPoses(goals_msg, behavior_tree="")
       if task is None:
           code, msg = navigator.getTaskError()
           navigator.error(f"goThroughPoses rejected: error_code={code}, error_msg='{msg}'")
           navigator.destroyNode()
           rclpy.shutdown()
           return

       last_log_time = 0.0
       while not navigator.isTaskComplete(task):
           feedback = navigator.getFeedback(task)
           now_sec = time.monotonic()
           if feedback and (now_sec - last_log_time) >= 0.5:
               navigator.info(
                   "Feedback: distance_remaining="
                   f"{feedback.distance_remaining:.2f}, "
                   f"poses_remaining={feedback.number_of_poses_remaining}, "
                   f"recoveries={feedback.number_of_recoveries}"
               )
               last_log_time = now_sec

       result = navigator.getResult()
       if result == TaskResult.SUCCEEDED:
           navigator.info("Navigation succeeded")
       elif result == TaskResult.CANCELED:
           navigator.warn("Navigation canceled")
       elif result == TaskResult.FAILED:
           code, msg = navigator.getTaskError()
           navigator.error(f"Navigation failed: error_code={code}, error_msg='{msg}'")
       else:
           code, msg = navigator.getTaskError()
           navigator.error(f"Navigation unknown result: error_code={code}, error_msg='{msg}'")

       navigator.destroyNode()
       rclpy.shutdown()


   if __name__ == "__main__":
       main()

Run:

.. code-block:: bash

   chmod +x ~/tb4_sim_ws/go_through_poses.py
   source /opt/ros/kilted/setup.bash
   source ~/tb4_sim_ws/install/setup.bash
   python3 ~/tb4_sim_ws/go_through_poses.py

Source code quick walkthrough
=============================

Key flow in ``nav2_minimal_tb4_sim/nav2_bt_navigator/src/navigators/navigate_through_poses.cpp``:

- ``configure()`` (line 30): declares blackboard IDs and runtime params, wires odom smoother, optional Groot monitoring.
- ``goalReceived()`` (line 83): loads BT from ``goal->behavior_tree``, then calls ``initializeGoalPoses()``.
- ``initializeGoalPoses()`` (line 248): transforms each goal pose to global frame, resets state, fills blackboard (``goals``, ``path``, ``waypoint_statuses``).
- ``onLoop()`` (line 133): publishes feedback (distance/time remaining, current pose, recoveries, tracking error, waypoint statuses).
- ``onPreempt()`` (line 219): accepts preemption only when requested BT is the same as current/default BT.
- ``goalCompleted()`` (line 96): finalizes remaining waypoint statuses as ``COMPLETED`` or ``FAILED``.

Useful behavior for demos:

- Send another goal while first is running (same BT) to exercise ``onPreempt()``.
- Watch waypoint status changes in action feedback.

Parameter naming note from this source:

- ``search_window_`` is declared with key ``navigate_through_posessearch_window`` (no ``.``) in ``configure()``.
- Blackboard IDs use dotted keys, for example ``navigate_through_poses.goals_blackboard_id``.

Troubleshooting
===============

- ``Unknown package nav2_msgs``:
  Nav2 is not installed/sourced in your environment.
- ``Error loading BT``:
  Verify ``behavior_tree`` path exists and points to installed share path.
- TF errors (``Initial robot pose is not available``):
  Wait for ``/tf`` + ``/clock``, confirm ``use_sim_time:=True`` everywhere.
- Robot does not move:
  Ensure Nav2 servers are active and action server exists:
  ``ros2 action list | rg navigate``.
- ROS 2 Kilted message type mismatch:
  ``NavigateThroughPoses.Goal.poses`` is ``nav_msgs/Goals`` (header + goals),
  not a plain ``PoseStamped`` list. Populate ``goal.poses.header`` and
  ``goal.poses.goals`` explicitly.
- If testing with Method 3 with Python Simple Commander, check the following:

  - startup blocks:
    ``waitUntilNav2Active(localizer='amcl')`` waits for ``amcl/get_state`` and
    ``amcl_pose``. If AMCL is not running, this will block.
    Use the correct localizer mode for your stack and verify ``bt_navigator``
    is active (``ros2 lifecycle get /bt_navigator``).

  - ``goThroughPoses()`` fails on ``len()``:
    A known branch-version mismatch may trigger a ``TypeError`` when evaluating
    ``len(goal_msg.poses)`` in ``robot_navigator.py``. As of February 22, 2026,
    upstream ``jazzy``/``kilted`` branch heads still show this line, while
    upstream ``main`` (Rolling development) uses ``len(poses.goals)``.
    Verify your installed file first, for example:
    ``rg -n "Navigating with .*goals" /opt/ros/$ROS_DISTRO/lib/python*/site-packages/nav2_simple_commander/robot_navigator.py``.
    If your installed file still has ``len(goal_msg.poses)``, temporarily comment
    out that log line or change it to ``len(poses.goals)``.
