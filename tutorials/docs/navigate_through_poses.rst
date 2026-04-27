.. _navigate_through_poses:


Navigating with Pose Constraints (Navigate Through Poses)
*********************************************************

- `Requirements`_
- `Start TB4 simulation and Nav2`_
- `Optional: Use Smac Planner`_
- `What is navigating through poses?`_
- `Behavior tree overview`_
- `Waypoint Follower vs. Navigate Through Poses`_
- `Send a goal in Python using Simple Commander`_
- `Demo video`_
- `References`_


This tutorial is for new users who want to:

1. Send a ``NavigateThroughPoses`` action goal.
2. Understand which ``NavigateThroughPosesNavigator`` functions are used at runtime.

Requirements
=============

- It is assumed that ROS 2 and Nav2 dependencies are already installed or built locally. Please refer to :ref:`getting_started`.

Start TB4 simulation and Nav2
=============================

In terminal:

.. code-block:: bash

   ros2 launch nav2_bringup tb4_simulation_launch.py \
     use_sim_time:=True \
     slam:=False \
     autostart:=True \
     headless:=False \
     map:=/opt/ros/${ROS_DISTRO}/share/nav2_bringup/maps/depot.yaml

Notes:

- This combined launch starts the TurtleBot 4 simulation and the Nav2 stack together.
- ``headless:=False`` opens the Gazebo GUI. Without it, the simulation runs in
  the background and only RViz is shown.
- The TB4 simulation assets are provided by ``nav2_minimal_tb4_sim``, installed
  under ``/opt/ros/${ROS_DISTRO}/share/nav2_minimal_tb4_sim``.
- ``slam:=False`` uses localization against the provided static map. This is
  preferred for this tutorial because the robot starts in a known map frame.
- ``tb4_simulation_launch.py`` uses ``x_pose`` and ``y_pose`` to spawn the
  robot in Gazebo. These are simulation world coordinates, not necessarily the
  AMCL initial pose coordinates in the loaded map.
- AMCL needs an initial pose before it can publish the ``map`` to ``odom``
  transform. After the launch file starts, set the robot's initial pose in RViz
  with the ``2D Pose Estimate`` tool before running the Python script.
- ``bringup_launch.py`` starts Nav2, but does not by itself spawn the TB4
  simulation. Use ``tb4_simulation_launch.py`` for this tutorial so the
  simulator, robot, RViz, and Nav2 stack are launched together.

Optional: Use Smac Planner
==========================

``NavigateThroughPoses`` can use any planner plugin loaded by the planner
server. For a TurtleBot-style simulation, ``SmacPlanner2D`` is a good choice for a
smooth, cost-aware demonstration.

The default Nav2 parameters file is installed at
``/opt/ros/${ROS_DISTRO}/share/nav2_bringup/params/nav2_params.yaml`` and uses
``nav2_navfn_planner::NavfnPlanner`` for the default ``GridBased`` planner.
Rather than editing the file in ``/opt/ros`` directly, copy it into a
``config`` directory in your own ROS 2 package, edit the copy, and pass that
file to the launch command. In the copied file, configure the planner server so
that ``GridBased`` maps to ``nav2_smac_planner::SmacPlanner2D``:

.. code-block:: yaml

   planner_server:
     ros__parameters:
       planner_plugins: ["GridBased"]

       GridBased:
         plugin: "nav2_smac_planner::SmacPlanner2D"
         tolerance: 0.125
         downsample_costmap: false
         downsampling_factor: 1
         allow_unknown: true
         max_iterations: 1000000
         max_on_approach_iterations: 1000
         max_planning_time: 2.0
         cost_travel_multiplier: 2.0
         use_final_approach_orientation: false
         smoother:
           max_iterations: 1000
           w_smooth: 0.3
           w_data: 0.2
           tolerance: 1.0e-10

Then launch Nav2 with the updated parameters file:

.. code-block:: bash

   ros2 launch nav2_bringup tb4_simulation_launch.py \
     use_sim_time:=True \
     slam:=False \
     autostart:=True \
     headless:=False \
     map:=/opt/ros/${ROS_DISTRO}/share/nav2_bringup/maps/depot.yaml \
     params_file:=/path/to/ros2_pkg/config/nav2_params.yaml

What is navigating through poses?
=================================

Navigating through poses means sending Nav2 an ordered list of target poses
instead of a single final destination. The robot drives to each pose in order,
using the ``NavigateThroughPoses`` action to plan, recover, and report feedback
for the full route. This is useful when an application needs the robot to visit
several waypoints, follow a preferred corridor, inspect multiple locations, or
break a larger navigation task into smaller intentional steps.

The `NavigateThroughPoses action definition <https://api.nav2.org/actions/kilted/navigatethroughposes.html>`_
describes the goal, result, and feedback messages used by this behavior. The
goal contains the ordered poses and an optional behavior tree override. The
result provides an error code and human-readable error message when navigation
does not succeed. The feedback is especially useful for monitoring progress
because it reports values such as the robot's current pose, navigation time,
estimated time remaining, number of recoveries, distance remaining, and number
of poses remaining. The Python example below logs ``distance_remaining`` and
``number_of_poses_remaining`` from that feedback message.

In Simple Commander, ``BasicNavigator.goThroughPoses(...)`` is the convenience
API for this behavior. It waits for the ``NavigateThroughPoses`` action server,
packages the provided ``nav_msgs/Goals`` message into a
``NavigateThroughPoses`` goal, sends it asynchronously, and returns a running
task handle when the request is accepted. The examples below use that task
handle to monitor feedback such as distance remaining, poses remaining, and
recoveries until navigation completes.

Behavior tree overview
======================

The default ``NavigateThroughPoses`` behavior tree plans with
``ComputePathThroughPoses``, prunes completed goals with ``RemovePassedGoals``,
and follows the generated path with ``FollowPath``. This is what lets Nav2 treat
the poses as constraints along one continuous route instead of separate
per-waypoint tasks.

See the full
`Navigate Through Poses behavior tree <https://docs.nav2.org/behavior_trees/trees/nav_through_poses_recovery.html>`_
for the complete XML with replanning, planner and controller recovery, and
system-level recovery actions.

Waypoint Follower vs. Navigate Through Poses
============================================

Use ``NavigateThroughPoses`` when the intermediate poses are part of one
continuous navigation task. The route is planned through multiple pose
constraints, and the robot is not expected to stop and execute a separate
arrival task at every intermediate pose. In Python, this is the behavior exposed
by ``BasicNavigator.goThroughPoses(...)``.

Use the Nav2 Waypoint Follower when each waypoint is a place where the robot
should arrive and optionally run work, such as waiting, taking a picture,
performing an inspection, or picking up an object. The Waypoint Follower uses
``NavigateToPose`` goals in order and hosts waypoint task executor plugins, so
the robot may slow down or stop as it reaches each waypoint. In Python, this
matches ``BasicNavigator.followWaypoints(...)`` and
``BasicNavigator.followGpsWaypoints(...)``. GPS waypoint following accepts GPS
waypoints and converts them into the global navigation frame, but its task
semantics are still waypoint-following semantics.

As a rule of thumb, choose Waypoint Follower when the waypoints are destinations
with per-waypoint work. Choose ``NavigateThroughPoses`` when the poses are
intermediate constraints along a smoother route.

Send a goal in Python using Simple Commander
============================================

This method uses ``nav2_simple_commander`` and sends ``nav_msgs/Goals`` to
``BasicNavigator.goThroughPoses(...)``.

The goal poses below are example ``map`` frame poses. If you use a different
map, choose poses in free space within that map's bounds.


Create ``~/tb4_sim_ws/go_through_poses.py``:

.. code-block:: python

   #!/usr/bin/env python3

   import time

   import rclpy
   from rclpy.duration import Duration
   from builtin_interfaces.msg import Time
   from geometry_msgs.msg import PoseStamped
   from nav_msgs.msg import Goals
   from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


   def make_pose(stamp: Time, x: float, y: float, w: float = 1.0) -> PoseStamped:
       pose = PoseStamped()
       pose.header.frame_id = "map"
       pose.header.stamp = stamp
       pose.pose.position.x = x
       pose.pose.position.y = y
       pose.pose.orientation.w = w
       return pose


   def main() -> None:
       rclpy.init()
       navigator = BasicNavigator(node_name="navigate_through_poses_py_client")

       navigator.waitUntilNav2Active()

       stamp = navigator.get_clock().now().to_msg()
       goals_msg = Goals()
       goals_msg.header.frame_id = "map"
       goals_msg.header.stamp = stamp
       goals_msg.goals = [
           make_pose(stamp, 7.0, 6.0, 1.0),
           make_pose(stamp, 12.0, 6.5, 1.0),
           make_pose(stamp, 9.5, 8.0, 1.0),
           make_pose(stamp, 5.0, 8.0, 1.0),
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
           if feedback and (now_sec - last_log_time) >= 1.0:
               navigator.info(
                   "Feedback: distance_remaining="
                   f"{feedback.distance_remaining:.2f}, "
                   f"poses_remaining={feedback.number_of_poses_remaining}, "
                   f"recoveries={feedback.number_of_recoveries}, "
                   "Estimated time of arrival: "
                   f"{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.0f}"
                   " seconds."
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

Demo video
==========

A video demonstration shows the robot executing this script with
``NavigateThroughPoses`` and Smac Planner configured for ``GridBased``. In RViz,
the planned path should pass through the ordered pose constraints and the robot
should continue through intermediate poses as part of one navigation task,
rather than stopping to run a separate waypoint task at each pose.

References
==========

- `Smac Planner Configuration Guide <https://docs.nav2.org/configuration/packages/configuring-smac-planner.html>`_
- `Nav2 Simple Commander Navigate Through Poses example <https://github.com/ros-navigation/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_nav_through_poses.py>`_
- `Achieving Smooth Navigation with Nav2 Waypoint Follower <https://robotics.stackexchange.com/questions/104201/achieving-smooth-navigation-with-nav2-waypoint-follower>`_
