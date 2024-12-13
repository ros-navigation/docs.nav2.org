.. _jazzy_migration:

Jazzy to K-Turtle
#################

Moving from ROS 2 Jazzy to K-Turtle, a number of stability improvements were added that we will not specifically address here.

TwistStamped Default CmdVel Change
**********************************

In Kilted and newer, the default ``cmd_vel`` topic for all ``Twist`` publishers and subscriptions is changed to ``TwistStamped`` in order to enable a broader range of applications.
it also allows for rejection of stale velocity messages, which can be useful in some applications.
Your robot should now subscribe to a ``TwistStamped`` message instead of a ``Twist`` message & update your simulation appropriately.
The topic names are the same.

However, this can be disabled by setting ``enable_twist_stamped`` to ``false`` in the ``nav2_params.yaml`` file for all nodes that involve Twist subscriptions or publications.
See the configuration guide for more information on how to configure this parameter for each node.

An example simulation migration using Gazebo can be seen in the `following pull request for the Turtlebot 3 and 4 <https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation/pull/16>`_.


New Nav2 Loopback Simulator
***************************

The ``nav2_loopback_sim`` is a stand-alone simulator to create a "loopback" for non-physical simulation to replace robot hardware, physics simulators (Gazebo, Bullet, Isaac Sim, etc).
It computes the robot's odometry based on the command velocity's output request to create a perfect 'frictionless plane'-style simulation for unit testing, system testing, R&D on higher level systems, testing behaviors without concerning yourself with localization accuracy or system dynamics, and multirobot simulations.

Docking with Static Infrastructure or Dynamic Docking
*****************************************************

In `PR #4627 <https://github.com/ros-navigation/navigation2/pull/4627>`_ a docking plugin type and logic was added to support non-charging dock types in the nav2 docking server.
This allows users to specify docking locations to static infrastructure, such as conveyers, or dynamic docking locations, such as a pallet.
It also includes a new docking plugin to demonstrate the new docking server capabilities ``simple_non_charging_dock``.

New RViz panel for Docking
**************************

In `PR #4458 <https://github.com/ros-navigation/navigation2/pull/4458>`_ a new RViz panel was added to interact with the Docking Server.

This panel allows the user to:

- Dock the robot to a charger, selecting the dock id or the dock pose and type. It also allows the user to set whether or not to navigate to the staging pose.
- Undock the robot from a charger.

The panel displays the action goal status and feedback published by ``nav2_msgs/DockRobot`` and ``nav2_msgs/UndockRobot`` actions. Users can find information such as elapsed time, number of retries and the current state of the action (staging, controlling, etc.), as well as the error codes of the action.


Here we can see the working demo of the plugin:

.. image:: images/docking_panel.gif

.. attention:: If the docking server is unavailable, then the combo box of the dock type will be empty.

New BT Nodes
************

Below is a list of new BT Nodes added:

- ``GetPoseFromPath``: An action to get a particular pose from an input path.
- ``RemoveInCollisionGoals``: An action to remove waypoints that have a cost higher than a threshold.
- ``IsStopped``: A condition to check if the robot is stopped for a certain duration.

New RViz Tool for Costmap Cost Cell Inspection
**********************************************

In `PR #4546 <https://github.com/ros-navigation/navigation2/pull/4546>`_ a new RViz tool was added to get the costmap costcell's cost and a service to get the costcell's cost at the footprint pose.

Usage:

- Click on any point in the costmap with costmap rviz tool to retrieve and display the cost value at that cell.
- ``nav2_msgs/GetCosts`` service can be used to retrieve the cost at footprint pose

Working demo of the tool:

.. image:: images/rviz_costmap_cost_tool.gif

.. attention:: If the costmap service is unavailable, then the tool will not be able to fetch and display the cost values.

Fix flickering visualization
****************************

In `PR #4561 <https://github.com/ros-navigation/navigation2/pull/4561>`_ a ``map_vis_z`` parameter has been introduced to Costmap2DROS to help modify the map slightly below the default plane, aiming to eliminate rviz visualization flickering issues.

Default Value:

- map_vis_z: 0.0

Minimum Value Without Flickering:

- map_vis_z: -0.008

Before:

.. image:: images/fix_flickering_visualization_before.png

After:

.. image:: images/fix_flickering_visualization_after.png

Option to limit velocity through DWB trajectory
***********************************************

In `PR #4663 <https://github.com/ros-navigation/navigation2/pull/4663>`_ a ``limit_vel_cmd_in_traj`` parameter was introduced to DWB local planner to allow the user to limit the velocity used in the trajectory generation based on the robot's current velocity.

Default value: 

- false

Option to disable zero velocity publishing on goal exit
*******************************************************

In `PR #4675 <https://github.com/ros-navigation/navigation2/pull/4675>`_ a ``publish_zero_velocity`` parameter was introduced for the `Controller server </configuration/packages/configuring-controller-server.html#controller-server>`_ in order to disable zero velocity publishing on goal exit.

Default value:

- true

Added optional collision checking for the Docking Server
********************************************************

In `PR #4752 <https://github.com/ros-navigation/navigation2/pull/4752>`_ an optional collision checking feature was added to the `Docking server </configuration/packages/configuring-docking-server.html#docking-server>`_ to check for collisions between the robot and the dock.

Default value:

- true

Revamped multirobot bringup and config files to use namespaces
**************************************************************

In `PR #4715 <https://github.com/ros-navigation/navigation2/pull/4715>`_ multirobot bringup and the use of namespaces were overhauled to be compatible out of the box with ROS namespaces and remove custom logic, specifically:

* The ``use_namespace`` parameter has been removed from ``nav2_bringup`` launch files. The ``namespace`` parameter will now always be used and default to ``/`` for "global namespace".
* There is now a single rviz config file for both normal and namespaced robots. Topics have been changed to a relative path (i.e. ``/map`` -> ``map``) and the rviz ``namespace`` will be added automatically.
* There is now a single ``nav2_params.yaml`` config file for both single and multirobot bringup. All the topics have been changed to relative (i.e. ``/scan`` -> ``scan``).

Note that some plugins / nodes might have their own local namespace. This is the case for ``CostmapLayer`` which will be in a ``/ns/[layer_name]`` namespace. For these, a new function ``joinWithParentNamespace`` has been added to make sure joining relative paths results in ``/ns/topic_name`` rather than ``/ns/[layer_name]/topic_name``.

If your use case doesn't require multiple robots, keeping absolute paths in your ``nav2_params.yaml`` config file and rviz config file will preserve existing behavior.

For example, if you specify ``topic: scan`` in the ``voxel_layer`` of a ``local_costmap`` and you launch your bringup with a ``tb4`` namespace:

* User chosen namespace is ``tb4``.
* User chosen topic is ``scan``.
* Topic will be remapped to ``/tb4/scan`` without ``local_costmap``.
* Use global topic ``/scan`` if you do not wish the node namespace to apply

Removed global map_topic from Costmap node
******************************************

In `PR #4715 <https://github.com/ros-navigation/navigation2/pull/4715>`_ the global ``map_topic`` parameter has been removed from the ``Costmap2DROS`` node. This parameterwas only used in the ``StaticLayer`` and should be defined as a parameter local to the ``StaticLayer`` instead, for example:

.. code-block:: yaml

  global_costmap:
    global_costmap:
      ros__parameters:
        [...]
        # Not supported anymore
        map_topic: my_map
        static_layer:
          plugin: "nav2_costmap_2d::StaticLayer"
          map_subscribe_transient_local: True
          # Do this instead
          map_topic: my_map

Simplified Costmap2DROS constructors
************************************

The following constructors for ``Costmap2DROS`` have been removed:

.. code-block:: cpp

   explicit Costmap2DROS(
    const std::string & name,
    const std::string & parent_namespace,
    const std::string & local_namespace,
    const bool & use_sim_time);

   explicit Costmap2DROS(const std::string & name, const bool & use_sim_time = false);

They have been consolidated into a single one:

.. code-block:: cpp

   explicit Costmap2DROS(
    const std::string & name,
    const std::string & parent_namespace = "/",
    const bool & use_sim_time = false);

The ``local_namespace`` parameter has been removed and is now automatically set to the node's name (which is what the second removed constructor did).
Parameters ``parent_namespace`` / ``use_sim_time`` both provide default values to maintain the ability of creating a ``Costmap2DROS`` object by just specifying a name.

Option to disable collision checking in DriveOnHeading, BackUp and Spin Actions
*******************************************************************************

In `PR #4785 <https://github.com/ros-navigation/navigation2/pull/4785>`_ a new boolean parameter named `disable_collision_checks` was added to the `DriveOnHeading`, `BackUp` and `Spin` actions to optionally disable collision checking. 
This can be useful, for example, in cases where you want to move the robot even in the presence of known obstacles.

Default value:

- false

Extended GoalChecker Interface
******************************

In `PR #4789 <https://github.com/ros-navigation/navigation2/pull/4789>`_ the GoalChecker interface gained a new parameter `path` in the method `is_goal_reached`.

Before:

.. code-block:: cpp
  
  bool isGoalReached(
      const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose
      const geometry_msgs::msg::Twist & velocity);
  
After:

.. code-block:: cpp
  
  bool isGoalReached(
      const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
      const geometry_msgs::msg::Twist & velocity, const nav_msgs::msg::Path & current_path);
  
This allows for GoalChecker plugins to account for the current path in deciding if the goal has been reached or not.

The new argument `current_path` can be ignored by existing out of tree plugins that do not make use of it, once they are updated to use the new method signature.

New (default) Goal Checker Plugin: PathCompleteGoalChecker
********************************************************

In `PR #4789 <https://github.com/ros-navigation/navigation2/pull/4789>`_ a new goal checker plugin: :ref:`configuring_nav2_controller_path_complete_goal_checker_plugin` was introduced.

This was made the default goal checker, resolving a `premature path completion bug <https://github.com/ros-navigation/navigation2/issues/4304>`_.
