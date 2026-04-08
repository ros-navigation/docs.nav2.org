.. _navigate_through_poses:


Navigating with Pose Constraints (Navigate Through Poses)
*********************************************************

- `Requirements`_
- `Start TB4 simulation and Nav2`_
- `Send a goal in Python using Simple Commander`_


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

   ros2 launch nav2_bringup tb4_simulation_launch.py slam:=True

Notes:

- This combined launch starts the TurtleBot 4 simulation and the Nav2 stack together.
- ``slam:=True`` starts SLAM for map creation and localization in a fresh simulation.
- With ``slam:=False``, Nav2 runs localization instead and expects ``map:=...`` to be provided.

Send a goal in Python using Simple Commander
============================================

This method uses ``nav2_simple_commander`` and sends ``nav_msgs/Goals`` to
``BasicNavigator.goThroughPoses(...)``.


Create ``~/tb4_sim_ws/go_through_poses.py``:

.. code-block:: python

   #!/usr/bin/env python3

   import time

   import rclpy
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

       # If you launched Nav2 with slam:=True, use localizer='robot_localization'.
       # For AMCL localization mode, use waitUntilNav2Active() with defaults.
       navigator.waitUntilNav2Active(localizer="robot_localization")

       stamp = navigator.get_clock().now().to_msg()
       goals_msg = Goals()
       goals_msg.header.frame_id = "map"
       goals_msg.header.stamp = stamp
       goals_msg.goals = [
           make_pose(stamp, 3.5, 0.0, 1.0),
           make_pose(stamp, 5.0, 1.0, 0.25),
           make_pose(stamp, 7.0, 2.0, 0.0),
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
