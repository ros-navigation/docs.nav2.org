.. _commander_api:

Simple Commander API
####################

Overview
********

The goal of the Nav2 Simple (Python3) Commander is to provide a "navigation as a library" capability to Python3 users. We provide an API that handles all the ROS 2 and Action Server tasks for you such that you can focus on building an application leveraging the capabilities of Nav2 (after you've configured it to your liking with your plugins of choice). `We also provide you with demos and examples of API usage <https://github.com/ros-planning/navigation2/tree/main/nav2_simple_commander>`_ to build common basic capabilities in autonomous mobile robotics in the ``nav2_simple_commander`` package.

A simple demonstration is shown below. Note: ``goToPose()``, ``goThroughPoses()``, ``followWaypoints()`` and similar are **non-blocking** such that you can receive and process feedback in a single-threaded application. As such while waiting for a task to be completed, the ``while not nav.isTaskComplete()`` design is necessary to poll for changes in the navigation completion, and if not complete some tasks of interest to your application (like processing feedback, doing something with the data the robot is collecting, or checking for faults).

You may use this simple commander preempt commands of the same type (e.g. you can preempt a ``goToPose()`` with another ``goToPose()``) but you must explicitly cancel a current command and issue a new one if switching between ``goToPose()``, ``goThroughPoses()``, or ``followWaypoints()``.

.. code-block:: python3

  from nav2_simple_commander.robot_navigator import BasicNavigator
  import rclpy

  rclpy.init()
  nav = BasicNavigator()
  
  # ...
  
  nav.setInitialPose(init_pose)
  nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()
  
  # ...
  
  nav.goToPose(goal_pose)
  while not nav.isTaskComplete():
    feedback = nav.getFeedback()
    if feedback.navigation_duration > 600:
      nav.cancelTask()
  
  # ...
  
  result = nav.getResult()
  if result == TaskResult.SUCCEEDED:
      print('Goal succeeded!')
  elif result == TaskResult.CANCELED:
      print('Goal was canceled!')
  elif result == TaskResult.FAILED:
      print('Goal failed!')


Commander API
*************

The methods provided by the basic navigator are shown below, with inputs and expected returns.
If a server fails, it may throw an exception or return a `None` object, so please be sure to properly wrap your navigation calls in try/catch and check returns for `None` type.

+---------------------------------------+----------------------------------------------------------------------------+
| Robot Navigator Method                | Description                                                                |
+=======================================+============================================================================+
| setInitialPose(initial_pose)          | Sets the initial pose (``PoseStamped``) of the robot to localization.      |
+---------------------------------------+----------------------------------------------------------------------------+
| goThroughPoses(poses,                 | Requests the robot to drive through a set of poses                         |
| behavior_tree='')                     | (list of ``PoseStamped``).                                                 |
+---------------------------------------+----------------------------------------------------------------------------+
| goToPose(pose, behavior_tree='')      | Requests the robot to drive to a pose (``PoseStamped``).                   |
+---------------------------------------+----------------------------------------------------------------------------+
| followWaypoints(poses)                | Requests the robot to follow a set of waypoints (list of ``PoseStamped``). |
|                                       | This will execute the chosen ``TaskExecutor`` plugin at each pose.         |
+---------------------------------------+----------------------------------------------------------------------------+
| followPath(path, controller_id='',    | Requests the robot to follow a path from a starting to a goal              |
| goal_checker_id='')                   | ``PoseStamped``, ``nav_msgs/Path``.                                        |
+---------------------------------------+----------------------------------------------------------------------------+
| spin(spin_dist=1.57,                  | Requests the robot to performs an in-place rotation by a given angle.      |
| time_allowance=10)                    |                                                                            |
+---------------------------------------+----------------------------------------------------------------------------+
| backup(backup_dist=0.15,              | Requests the robot to back up by a given distance.                         |
| backup_speed=0.025, time_allowance=10)|                                                                            |
+---------------------------------------+----------------------------------------------------------------------------+
| cancelTask()                          | Cancel an ongoing task.                                                    |
+---------------------------------------+----------------------------------------------------------------------------+
| isTaskComplete()                      | Checks if task is complete yet, times out at ``100ms``. Returns            |
|                                       | ``True`` if completed and ``False`` if still going.                        |
+---------------------------------------+----------------------------------------------------------------------------+
| getFeedback()                         | Gets feedback from task, returns action server feedback msg.               |
+---------------------------------------+----------------------------------------------------------------------------+
| getResult()                           | Gets final result of task, to be called after ``isTaskComplete``           |
|                                       | returns ``True``. Returns action server result msg.                        |
+---------------------------------------+----------------------------------------------------------------------------+
| getPath(start, goal,                  | Gets a path from a starting to a goal ``PoseStamped``, ``nav_msgs/Path``.  |
| planner_id='', use_start=False)       |                                                                            |
+---------------------------------------+----------------------------------------------------------------------------+
| getPathThroughPoses(start, goals,     | Gets a path through a starting to a set of goals, a list                   |
| planner_id='', use_start=False)       | of ``PoseStamped``, ``nav_msgs/Path``.                                     |
+---------------------------------------+----------------------------------------------------------------------------+
| changeMap(map_filepath)               | Requests a change from the current map to `map_filepath`'s yaml.           |
+---------------------------------------+----------------------------------------------------------------------------+
| clearAllCostmaps()                    | Clears both the global and local costmaps.                                 |
+---------------------------------------+----------------------------------------------------------------------------+
| clearLocalCostmap()                   | Clears the local costmap.                                                  |
+---------------------------------------+----------------------------------------------------------------------------+
| clearGlobalCostmap()                  | Clears the global costmap.                                                 |
+---------------------------------------+----------------------------------------------------------------------------+
| getGlobalCostmap()                    | Returns the global costmap, ``nav2_msgs/Costmap``.                         |
+---------------------------------------+----------------------------------------------------------------------------+
| getLocalCostmap()                     | Returns the local costmap, ``nav2_msgs/Costmap``.                          |
+---------------------------------------+----------------------------------------------------------------------------+
| waitUntilNav2Active(                  | Blocks until Nav2 is completely online and lifecycle nodes are in the      |
| navigator='bt_navigator',             | active state. To be used in conjunction with autostart or external         |
| localizer='amcl')                     | lifecycle bringup. Custom navigator and localizer nodes can be specified   |
+---------------------------------------+----------------------------------------------------------------------------+
| lifecycleStartup()                    | Sends a request to all lifecycle management servers to bring them into     |
|                                       | the active state, to be used if autostart is ``False`` and you want this   |
|                                       | program to control Nav2's lifecycle.                                       |
+---------------------------------------+----------------------------------------------------------------------------+
| lifecycleShutdown()                   | Sends a request to all lifecycle management servers to shut them down.     |
+---------------------------------------+----------------------------------------------------------------------------+

Examples and Demos
******************

All of these can be found in the `package <https://github.com/ros-planning/navigation2/tree/main/nav2_simple_commander>`_.

.. image:: readme.gif
  :width: 800
  :alt: Alternative text
  :align: center

The ``nav2_simple_commander`` has a few examples to highlight the API functions available to you as a user:

- ``example_nav_to_pose.py`` - Demonstrates the navigate to pose capabilities of the navigator, as well as a number of auxiliary methods.
- ``example_nav_through_poses.py`` - Demonstrates the navigate through poses capabilities of the navigator, as well as a number of auxiliary methods.
- ``example_waypoint_follower.py`` - Demonstrates the waypoint following capabilities of the navigator, as well as a number of auxiliary methods.
- ``example_follow_path.py`` - Demonstrates the path following capabilities of the navigator, as well as a number of auxiliary methods.

The ``nav2_simple_commander`` has a few demonstrations to highlight a couple of simple autonomy applications you can build using the API:

- ``demo_security.py`` - A simple security robot application, showing how to have a robot follow a security route using Navigate Through Poses to do a patrol route, indefinitely. 
- ``demo_picking.py`` - A simple item picking application, showing how to have a robot drive to a specific shelf in a warehouse to either pick an item or have a person place an item into a basket and deliver it to a destination for shipping using Navigate To Pose.
- ``demo_inspection.py`` - A simple shelf inspection application, showing how to use the Waypoint Follower and task executors to take pictures, RFID scans, etc of shelves to analyze the current shelf statuses and locate items in the warehouse.
