.. _foxy_migration:

Foxy to Galactic
################

Moving from ROS 2 Foxy to Galactic, a number of stability improvements were added that we will not specifically address here.

NavigateToPose Action Feedback updates
**************************************

The NavigateToPose action feedback has two improvements:

- ``distance_remaining`` now integrates path poses to report accurate distance remaining to go. Previously, this field reported the euclidean distance between the current pose and the goal pose.
- Addition of ``estimated_time_remaining`` field. This field reports the estimated time remaining by dividing the remaining distance by the current speed.

NavigateToPose BT-node Interface Changes
****************************************

The NavigateToPose input port has been changed to PoseStamped instead of Point and Quaternion.

See :ref:`bt_navigate_to_pose_action` for more information.


NavigateThroughPoses and ComputePathThroughPoses Actions Added
**************************************************************

The ``NavigateThroughPoses`` action has been added analog to the ``NavigateToPose``. Rather than going to a single position, this Action will allow a user to specify a number of hard intermediary pose constraints between the start and final pose to plan through. The new ``ComputePathThroughPoses`` action has been added to the ``planner_server`` to process these requests through ``N goal_poses``.

The ``ComputePathThroughPoses`` action server will take in a set of ``N`` goals to achieve, plan through each pose and concatenate the output path for use in navigation. The controller and navigator know nothing about the semantics of the generated path, so the robot will not stop or slow on approach to these goals. It will rather continue through each pose as it were any other point on the path continuously. When paired with the ``SmacPlanner``, this feature can be used to generate **completely kinematically feasible trajectories through pose constraints**. 

If you wish to stop at each goal pose, consider using the waypoint follower instead, which will stop and allow a user to optionally execute a task plugin at each pose. 

ComputePathToPose BT-node Interface Changes
*******************************************

The ``start`` input port has been added to optionally allow the request of a path from ``start`` to ``goal``  instead of from the current position of the robot to ``goal``.

See :ref:`bt_compute_path_to_pose_action` for more information.

ComputePathToPose Action Interface Changes
*******************************************

- The goal pose field ``pose`` was changed to ``goal``.
- The PoseStamped field ``start`` has been added.
- The bool field ``use_start`` has been added.

These two additional fields have been added to optionally allow, when ``use_start`` is true, the request of a path from ``start`` to ``goal`` instead of from the current position of the robot to ``goal``. Corresponding changes have been done of the Planner Server.

BackUp BT-node Interface Changes
********************************

The ``backup_dist`` and ``backup_speed`` input ports should both be positive values indicating the distance to go backward respectively the speed with which the robot drives backward.

BackUp Recovery Interface Changes
*********************************

``speed`` in a backup recovery goal should be positive indicating the speed with which to drive backward.
``target.x`` in a backup recovery goal should be positive indicating the distance to drive backward.
In both cases negative values are silently inverted.

Nav2 Controllers and Goal Checker Plugin Interface Changes
**********************************************************

As of `this PR 2247 <https://github.com/ros-planning/navigation2/pull/2247>`_, the ``controller`` plugins will now be given a pointer to the current goal checker in use of the navigation task in ``computeAndPublishVelocity()``. This is geared to enabling controllers to have access to predictive checks for goal completion as well as access to the state information of the goal checker plugin.

The ``goal_checker`` plugins also have the change of including a ``getTolerances()`` method. This method allows a goal checker holder to access the tolerance information of the goal checker to consider at the goal. Each field of the ``pose`` and ``velocity`` represents the maximum allowable error in each dimension for a goal to be considered completed. In the case of a translational tolerance (combined X and Y components), each the X and Y will be populated with the tolerance value because it is the **maximum** tolerance in the dimension (assuming the other has no error). If the goal checker does not contain any tolerances for a dimension, the ``numeric_limits<double> lowest()`` value is utilized in its place.

FollowPath goal_checker_id attribute
************************************
For example: you could use for some specific navigation motion a more precise goal checker than the default one that it is used in usual motions.

.. code-block:: xml

    <FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="precise_goal_checker" server_name="FollowPath" server_timeout="10"/>

- The previous usage of the ``goal_checker_plugin`` parameter to declare the controller_server goal_checker is now obsolete and removed.
- The controller_server parameters now support the declaration of a list of goal checkers ``goal_checker_plugins`` mapped to unique identifier names, such as is the case with ``FollowPath`` and ``GridBased`` for the controller and planner plugins, respectively. 

- The specification of the selected goal checker is mandatory when more than one checker is defined in the controller_server parameter configuration. If only one goal_checker is configured in the controller_server it is selected by default even if no goal_checker is specified.

Below it is shown an example of goal_checker configuration of the controller_server node.

.. code-block:: yaml

    controller_server:
      ros__parameters:
          goal_checker_plugins: ["general_goal_checker", "precise_goal_checker"]
          precise_goal_checker:
              plugin: "nav2_controller::SimpleGoalChecker"
              xy_goal_tolerance: 0.25
             yaw_goal_tolerance: 0.25
          general_goal_checker:
              plugin: "nav2_controller::SimpleGoalChecker"
              xy_goal_tolerance: 0.25




Groot Support
*************

Live Monitoring and Editing of behavior trees with Groot is now possible.
Switching bt-xmls on the fly through a new goal request is also included.
This is all done without breaking any APIs.
Enabled by default.

New Plugins
***********

``nav2_waypoint_follower`` has an action server that takes in a list of waypoints to follow and follow them in order. In some cases we might want robot to 
perform some tasks/behaviours at arrivals of these waypoints. In order to perform such tasks, a generic plugin interface `WaypointTaskExecutor` has been added to ``nav2_core``.
Users can inherit from this interface to implement their own plugin to perform more specific tasks at waypoint arrivals for their needs. 

Several example implementations are included in ``nav2_waypoint_follower``. ``WaitAtWaypoint`` and ``PhotoAtWaypoint`` plusings are included in 
``nav2_waypoint_follower`` as run-time loadable plugins. ``WaitAtWaypoint`` simply lets robot to pause for a specified amount of time in milliseconds, at waypoint arrivals.
While ``PhotoAtWaypoint`` takes photos at waypoint arrivals and saves the taken photos to specified directory, the format for taken photos also can be configured through parameters.
All major image formats such as ``png``, ``jpeg``, ``jpg`` etc. are supported, the default format is ``png``.

Loading a plugin of this type is done through ``nav2_bringup/params/nav2_param.yaml``, by specifying plugin's name, type and it's used parameters. 

.. code-block:: yaml

    waypoint_follower:
      ros__parameters:
        loop_rate: 20
        stop_on_failure: false
        waypoint_task_executor_plugin: "wait_at_waypoint"
          wait_at_waypoint:
            plugin: "nav2_waypoint_follower::WaitAtWaypoint"
            enabled: True
            waypoint_pause_duration: 0

Original GitHub tickets:

- `WaypointTaskExecutor <https://github.com/ros-planning/navigation2/pull/1993>`_
- `WaitAtWaypoint <https://github.com/ros-planning/navigation2/pull/1993>`_
- `PhotoAtWaypoint <https://github.com/ros-planning/navigation2/pull/2041>`_
- `InputAtWaypoint <https://github.com/ros-planning/navigation2/pull/2049>`_

Costmap Filters
***************

A new concept interacting with spatial-dependent objects called "Costmap Filters" appeared in Galactic (more information about this concept could be found at :ref:`concepts` page). Costmap filters are acting as a costmap plugins, applied to a separate costmap above common plugins. In order to make a filtered costmap and change robot's behavior in annotated areas, filter plugin reads the data came from filter mask. Then this data is being linearly transformed into feature map in a filter space. It could be passability of an area, maximum speed limit in m/s, robot desired direction in degrees or anything else. Transformed feature map along with the map/costmap, sensors data and current robot position is used in plugin's algorithms to make required updates in the resulting costmap and robot's behavor.

Architecturally, costmap filters consists from ``CostmapFilter`` class which is a basic class incorporating much common of its inherited filter plugins:

- ``KeepoutFilter``: keep-out/safety zones filter plugin.
- ``SpeedFilter``: slow/speed-restricted areas filter.
- Preferred lanes in industries. This plugin is covered by ``KeepoutFilter`` (see discussion in `corresponding PR <https://github.com/ros-planning/navigation2/issues/1522>`_ for more details).

Each costmap filter subscribes to filter info topic (publishing by `Costmap Filter Info Publisher Server <https://github.com/ros-planning/navigation2/tree/main/nav2_map_server/src/costmap_filter_info>`_) having all necessary information for loaded costmap filter and filter mask topic.
``SpeedFilter`` additionally publishes maximum speed restricting `messages <https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/msg/SpeedLimit.msg>`_ targeted for a Controller to enforce robot won't exceed given limit.

High-level design of this concept could be found `here <https://github.com/ros-planning/navigation2/tree/main/doc/design/CostmapFilters_design.pdf>`_. The functionality of costmap filters is being disscussed in `the ticket #1263 <https://github.com/ros-planning/navigation2/issues/1263>`_ and carried out by `PR #1882 <https://github.com/ros-planning/navigation2/pull/1882>`_. The following tutorials: :ref:`navigation2_with_keepout_filter` and :ref:`navigation2_with_speed_filter` will help to easily get involved with ``KeepoutFilter`` and ``SpeedFilter`` functionalities.

SmacPlanner
***********

A new package, ``nav2_smac_planner`` was added containing 4 or 8 connected 2D A*, and Dubin and Reed-shepp model hybrid-A* with smoothing, multi-resolution query, and more.

The ``nav2_smac_planner`` package contains an optimized templated A* search algorithm used to create multiple A*-based planners for multiple types of robot platforms. We support differential-drive and omni-directional drive robots using the ``SmacPlanner2D`` planner which implements a cost-aware A* planner. We support cars, car-like, and ackermann vehicles using the ``SmacPlanner`` plugin which implements a Hybrid-A* planner. This plugin is also useful for curvature constrained planning, like when planning robot at high speeds to make sure they don't flip over or otherwise skid out of control.

The ``SmacPlanner`` fully-implements the Hybrid-A* planner as proposed in `Practical Search Techniques in Path Planning for Autonomous Driving <https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf>`_, including hybrid searching, CG smoothing, analytic expansions and hueristic functions.

ThetaStarPlanner
****************
A new package, ``nav2_theta_star_planner`` was added containing 4 or 8 connected Theta* implementation for 2D maps.

This package implements an optimized version of the Theta* Path Planner (specifically the `Lazy Theta\* P <http://idm-lab.org/bib/abstracts/papers/aaai10b.pdf>`_ variant) to plan any-angled paths for differential-drive and omni-directional robots, while also taking into account the costmap costs. This plugin is useful for the cases where you might want to plan a path at a higher rate but without requiring extremely smooth paths around the corners which, for example, could be handled by a local planner/controller.

RegulatedPurePursuitController
******************************

A new package, ``nav2_regulated_pure_pursuit_controller`` was added containing a novel varient of the Pure Pursuit algorithm.
It also includes configurations to enable Pure Pursuit and Adaptive Pure Pursuit variations as well.

This variation is specifically targeting service / industrial robot needs.
It regulates the linear velocities by curvature of the path to help reduce overshoot at high speeds around blind corners allowing operations to be much more safe.
It also better follows paths than any other variation currently available of Pure Pursuit.
It also has heuristics to slow in proximity to other obstacles so that you can slow the robot automatically when nearby potential collisions.
It also implements the Adaptive lookahead point features to be scaled by velocities to enable more stable behavior in a larger range of translational speeds.

There's more this does, that that's the general information. See the package's ``README`` for more.

Costmap2D ``current_`` Usage
****************************

In costmap2D, ``current_`` was used in ROS1 to represent whether a costmap layer was still enabled and actively processing data. It would be turned to ``false`` only under the situation that the expected update rate of a sensor was not met, so it was getting stale or no messages. It acts as a fail-safe for if a navigation sensor stops publishing.

In galactic, that will remain turn, however it will also add additional capabilities. It is also now set to ``false`` when a costmap is reset due to clearing or other navigation recoveries. That stops the robot from creating a plan or control effort until after the costmap has been updated at least once after a reset. This enables us to make sure we cannot ever create a path or control with a completely empty costmap, potentially leading to collisions, due to clearing the costmap and then immediately requesting an algorithm to run.

Standard time units in parameters
*********************************
To follow the SI units outlined in REP-103 to the "T" nodes below were modified to use seconds consistently in every parameter. Under each node name you can see which parameters changed to seconds instead of using milliseconds.

- lifecycle manager 

  - ``bond_timeout_ms`` became ``bond_timeout`` in seconds

- smac planner

  - ``max_planning_time_ms`` became ``max_planning_time`` in seconds

- map saver

  - ``save_map_timeout`` in seconds

Ray Tracing Parameters
**********************
Raytracing functionality was modified to include a minimum range parameter from which ray tracing starts to clear obstacles to avoid incorrectly clearing obstacles too close to the robot. This issue was mentioned in `ROS Answers <https://answers.ros.org/question/355150/obstacles-in-sensor-deadzone/>`_. An existing parameter ``raytrace_range`` was renamed to ``raytrace_max_range`` to reflect the functionality it affects. The renamed parameters and the plugins that they belong to are mentioned below. The changes were introduced in this `pull request <https://github.com/ros-planning/navigation2/pull/2126>`_.

- obstacle_layer plugin

  - ``raytrace_min_range`` controls the minimum range from which ray tracing clears obstacles from the costmap

  - ``raytrace_max_range`` controls the maximum range to which ray tracing clears obstacles from the costmap

- voxel_layer plugin
 
  - ``raytrace_min_range`` controls the minimum range from which ray tracing clears obstacles from the costmap
 
  - ``raytrace_max_range`` controls the maximum range to which ray tracing clears obstacles from the costmap

Obstacle Marking Parameters
***************************
Obstacle marking was modified to include a minimum range parameter from which obstacles are marked on the costmap to prevent addition of obstacles in the costmap due to noisy and incorrect measurements. This modification is related to the change with the raytracing parameters. The renamed parameters, newly added parameters and the plugins they belong to are given below.

- obstacle_layer plugin

  - ``obstacle_min_range`` controls the minimum range from which obstacle are marked on the costmap

  - ``obstacle_max_range`` controls the maximum range to which obstacles are marked on the costmap

- voxel_layer plugin

  - ``obstacle_min_range`` controls the minimum range from which obstacle are marked on the costmap

  - ``obstacle_max_range`` controls the maximum range to which obstacles are marked on the costmap

Recovery Action Changes
***********************
The recovery actions, ``Spin`` and ``BackUp`` were modified to correctly return ``FAILURE`` if the recovery action is aborted due to a potential collision. Previously, these actions incorrectly always returned ``SUCCESS``. Changes to this resulted in downstream action clients, such as the default behavior tree. The changes were introduced in this `pull request 1855 <https://github.com/ros-planning/navigation2/pull/1855>`_.

Default Behavior Tree Changes
*****************************
The default behavior tree (BT) ``navigate_w_replanning_and_recovery.xml`` has been updated to allow for replanning in between recoveries. The changes were introduced in this `PR 1855 <https://github.com/ros-planning/navigation2/pull/1855>`_. Additionally, an alternative BT ``navigate_w_replanning_and_round_robin_recovery.xml`` was removed due to similarity with the updated default BT.

NavFn Planner Parameters
************************
The NavFn Planner has now its 3 parameters reconfigurable at runtime (``tolerance``, ``use_astar`` and ``allow_unknown``). The changes were introduced in this `pull request 2181 <https://github.com/ros-planning/navigation2/pull/2181>`_.

New ClearCostmapExceptRegion and ClearCostmapAroundRobot BT-nodes
*****************************************************************
The ClearEntireCostmap action node was already implemented but the ClearCostmapExceptRegion and ClearCostmapAroundRobot BT nodes calling the sister services ``(local_or_global)_costmap/clear_except_(local_or_global)_costmap`` and ``clear_around_(local_or_global)_costmap`` of Costmap 2D were missing, they are now implemented in a similar way. They both expose a ``reset_distance`` input port. See :ref:`bt_clear_costmap_except_region_action` and :ref:`bt_clear_entire_costmap_around_robot_action` for more.  The changes were introduced in this `pull request 2204 <https://github.com/ros-planning/navigation2/pull/2204>`_.

New Behavior Tree Nodes
***********************
A new behavior tree node was added and dynamically loadable at run-time using behavior tree cpp v3.
See ``nav2_behavior_tree`` for a full listing, or :ref:`plugins` for the current list of behavior tree plugins and their descriptions.
These plugins are set as default in the ``nav2_bt_navigator`` but may be overridden by the ``bt_plugins`` parameter to include your specific plugins.

Original GitHub tickets:

- `SingleTrigger <https://github.com/ros-planning/navigation2/pull/2236>`_
- `PlannerSelector <https://github.com/ros-planning/navigation2/pull/2249>`_
- `ControllerSelector <https://github.com/ros-planning/navigation2/pull/2266>`_
- `GoalCheckerSelector <https://github.com/ros-planning/navigation2/pull/2269>`_
- `NavigateThroughPoses <https://github.com/ros-planning/navigation2/pull/2271>`_
- `RemovePassedGoals <https://github.com/ros-planning/navigation2/pull/2271>`_
- `ComputePathThroughPoses <https://github.com/ros-planning/navigation2/pull/2271>`_

Additionally, behavior tree nodes were modified to contain their own local executors to spin for actions, topics, services, etc to ensure that each behavior tree node is independent of each other (e.g. spinning in one BT node doesn't trigger a callback in another). 

sensor_msgs/PointCloud to sensor_msgs/PointCloud2 Change
********************************************************
Due to deprecation of `sensor_msgs/PointCloud <https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud.html>`_ the topics which were publishing sensor_msgs/PointCloud are converted to sensor_msgs/PointCloud2. The details on these topics and their respective information are listed below.

- ``clearing_endpoints`` topic in ``voxel_layer`` plugin of ``nav2_costmap_2d`` package
- ``voxel_marked_cloud`` and ``voxel_unknown_cloud`` topic in ``costmap_2d_cloud`` node of ``nav2_costmap_2d`` package
- ``cost_cloud`` topic of ``publisher.cpp`` of ``dwb_core`` package.

These changes were introduced in `pull request 2263 <https://github.com/ros-planning/navigation2/pull/2263>`_.

ControllerServer New Parameter failure_tolerance
************************************************
A new parameter :code:`failure_tolerance` was added to the Controller Server for tolerating controller plugin exceptions without failing immediately. It is analogous to ``controller_patience`` in ROS(1) Nav. See :ref:`configuring_controller_server` for description.
This change was introduced in this `pull request 2264 <https://github.com/ros-planning/navigation2/pull/2264>`_.

Removed BT XML Launch Configurations
************************************
The launch python configurations for CLI setting of the behavior tree XML file has been removed. Instead, you should use the yaml files to set this value. If you, however, have a ``path`` to the yaml file that is inconsistent in a larger deployment, you can use the ``RewrittenYaml`` tool in your parent launch file to remap the default XML paths utilizing the ``get_shared_package_path()`` directory finder (or as you were before in python3).

The use of map subscription QoS launch configuration was also removed, use parameter file. 
This change was introduced in this `pull request 2295 <https://github.com/ros-planning/navigation2/pull/2295>`_.

Nav2 RViz Panel Action Feedback Information
*******************************************
The Nav2 RViz Panel now displays the action feedback published by ``nav2_msgs/NavigateToPose`` and ``nav2_msgs/NavigateThroughPoses`` actions.
Users can find information like the estimated time of arrival, distance remaining to goal, time elapsed since navigation started, and number of recoveries performed during a navigation action directly through the RViz panel.
This feature was introduced in this `pull request 2338 <https://github.com/ros-planning/navigation2/pull/2338>`_.

.. image:: /images/rviz/panel-feedback.gif
    :width: 600px
    :align: center
    :alt: Navigation feedback in RViz.
