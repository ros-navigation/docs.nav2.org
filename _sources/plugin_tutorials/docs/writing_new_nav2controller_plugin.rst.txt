.. _writing_new_nav2controller_plugin:

Writing a New Controller Plugin
*******************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

.. image:: images/Writing_new_nav2controller_plugin/nav2_pure_pursuit_gif.gif
    :width: 640px
    :align: center
    :alt: Animated gif of pure pursuit controller demo


Overview
========

This tutorial shows how to create your own controller `plugin <https://index.ros.org/p/pluginlib/>`_.

In this tutorial, we will be implementing the pure pursuit path tracking algorithm based on this `paper <https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf>`_. 
It is recommended you go through it.

Note: This tutorial is based on a previously existing simplified version of the Regulated Pure Pursuit controller now in the Nav2 stack.
You can find the source code matching this tutorial `here <https://github.com/ros-planning/navigation2_tutorials/tree/126902457c5c646b136569886d6325f070c1073d/nav2_pure_pursuit_controller>`_.

Requirements
============

- ROS 2 (binary or build-from-source)
- Nav2 (Including dependencies)
- Gazebo
- Turtlebot3

Tutorial Steps
==============

1- Create a new Controller Plugin
---------------------------------

We will be implementing the pure pursuit controller. The annotated code in this tutorial can be found in `navigation_tutorials <https://github.com/ros-planning/navigation2_tutorials>`_ repository 
as the ``nav2_pure_pursuit_controller``. This package can be considered as a reference for writing your own controller plugin.

Our example plugin class ``nav2_pure_pursuit_controller::PurePursuitController`` inherits from the base class ``nav2_core::Controller``. The base class provides a
set of virtual methods to implement a controller plugin. These methods are called at runtime by the controller server to compute velocity commands.
The list of methods, their descriptions, and necessity are presented in the table below:

+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| **Virtual method**        | **Method description**                                                                | **Requires override?** |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| configure()               | Method is called when controller server enters on_configure state. Ideally this       | Yes                    |
|                           | method should perform declarations of ROS parameters and initialization of            |                        |
|                           | controller's member variables. This method takes 4 input params: weak pointer to      |                        |
|                           | parent node, controller name, tf buffer pointer and shared pointer to costmap.        |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| activate()                | Method is called when controller server enters on_activate state. Ideally this method | Yes                    |
|                           | should implement operations which are neccessary before controller goes to an active  |                        |
|                           | state.                                                                                |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| deactivate()              | Method is called when controller server enters on_deactivate state. Ideally this      | Yes                    |
|                           | method should implement operations which are neccessary before controller goes to an  |                        |
|                           | inactive state.                                                                       |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| cleanup()                 | Method is called when controller server goes to on_cleanup state. Ideally this method | Yes                    |
|                           | should clean up resources which are created for the controller.                       |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| setPlan()                 | Method is called when the global plan is updated. Ideally this method should perform  | Yes                    |
|                           | operations that transform the global plan and store it.                               |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| computeVelocityCommands() | Method is called when a new velocity command is demanded by the controller server     | Yes                    |
|                           | in-order for the robot to follow the global path. This method returns a               |                        |
|                           | `geometry_msgs\:\:msg\:\:TwistStamped` which represents the velocity command for the  |                        |
|                           | robot to drive.  This method passes 2 parameters: reference to the current robot      |                        |
|                           | pose and its current velocity.                                                        |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| setSpeedLimit()           | Method is called when it is required to limit the maximum linear speed of the robot.  | Yes                    |
|                           | Speed limit could be expressed in absolute value (m/s) or in percentage from maximum  |                        |
|                           | robot speed. Note that typically, maximum rotational speed is being limited           |                        |
|                           | proportionally to the change of maximum linear speed, in order to keep current robot  |                        |
|                           | behavior untouched.                                                                   |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+

In this tutorial, we will use the methods ``PurePursuitController::configure``, ``PurePursuitController::setPlan`` and
``PurePursuitController::computeVelocityCommands``.

In controllers, ``configure()`` method must set member variables from ROS parameters and perform any initialization required.

.. code-block:: c++

  void PurePursuitController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent;
    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(
        0.2));
    declare_parameter_if_not_declared(
      node, plugin_name_ + ".lookahead_dist",
      rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(
      node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(
        1.0));
    declare_parameter_if_not_declared(
      node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(
        0.1));

    node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    double transform_tolerance;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
  }

Here, ``plugin_name_ + ".desired_linear_vel"`` is fetching the ROS parameter ``desired_linear_vel`` which is specific to our controller. 
Nav2 allows loading of multiple plugins, and to keep things organized, each plugin is mapped to some ID/name.
Now, if we want to retrieve the parameters for that specific plugin, we use ``<mapped_name_of_plugin>.<name_of_parameter>`` as done in the above snippet. 
For example, our example controller is mapped to the name ``FollowPath`` and to retrieve the ``desired_linear_vel`` parameter, which is specific to "FollowPath”, 
we used ``FollowPath.desired_linear_vel``. In other words, ``FollowPath`` is used as a namespace for plugin-specific parameters. 
We will see more on this when we discuss the parameters file (or params file).

The passed-in arguments are stored in member variables so that they can be used at a later stage if needed.

In ``setPlan()`` method, we receive the updated global path for the robot to follow. In our example, we transform the received global path into 
the frame of the robot and then store this transformed global path for later use.

.. code-block:: c++

  void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
  {
    // Transform global path into the robot's frame
    global_plan_ = transformGlobalPlan(path);
  }

The computation for the desired velocity happens in the ``computeVelocityCommands()`` method. It is used to calculate the desired velocity command given the current velocity and pose.
The third argument - is a pointer to the ``nav2_core::GoalChecker``, that checks whether a goal has been reached. In our example, this won't be used.
In the case of pure pursuit, the algorithm computes velocity commands such that the robot tries to follow the global path as closely as possible.
This algorithm assumes a constant linear velocity and computes the angular velocity based on the curvature of the global path.

.. code-block:: c++

  geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/)
  {
    // Find the first pose which is at a distance greater than the specified lookahead distance
    auto goal_pose = std::find_if(
      global_plan_.poses.begin(), global_plan_.poses.end(),
      [&](const auto & global_plan_pose) {
        return hypot(
          global_plan_pose.pose.position.x,
          global_plan_pose.pose.position.y) >= lookahead_dist_;
      })->pose;

    double linear_vel, angular_vel;

    // If the goal pose is in front of the robot then compute the velocity using the pure pursuit algorithm
    // else rotate with the max angular velocity until the goal pose is in front of the robot
    if (goal_pose.position.x > 0) {

      auto curvature = 2.0 * goal_pose.position.y /
        (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
      linear_vel = desired_linear_vel_;
      angular_vel = desired_linear_vel_ * curvature;
    } else {
      linear_vel = 0.0;
      angular_vel = max_angular_vel_;
    }

    // Create and publish a TwistStamped message with the desired velocity
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist.linear.x = linear_vel;
    cmd_vel.twist.angular.z = max(
      -1.0 * abs(max_angular_vel_), min(
        angular_vel, abs(
          max_angular_vel_)));

    return cmd_vel;
  }

The remaining methods are not used, but it's mandatory to override them. As per the rules, we did override all but left them empty.

2- Exporting the controller plugin
----------------------------------

Now that we have created our custom controller, we need to export our controller plugin so that it will be visible to the controller server. 
Plugins are loaded at runtime, and if they are not visible, then our controller server won't be able to load them. In ROS 2, exporting and loading 
plugins is handled by ``pluginlib``.

Coming back to our tutorial, class ``nav2_pure_pursuit_controller::PurePursuitController`` is loaded dynamically as ``nav2_core::Controller`` which is our base class.

1. To export the controller, we need to provide two lines

.. code-block:: c++
 
 #include "pluginlib/class_list_macros.hpp"
 PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)

Note that it requires pluginlib to export out the plugin's class. Pluginlib would provide as macro ``PLUGINLIB_EXPORT_CLASS``, which does all the work of exporting.

It is good practice to place these lines at the end of the file, but technically, you can also write at the top.

2. The next step would be to create the plugin's description file in the root directory of the package. For example, ``pure_pursuit_controller_plugin.xml`` file in our tutorial package. This file contains the following information

- ``library path``: Plugin's library name and its location.
- ``class name``: Name of the class.
- ``class type``: Type of class.
- ``base class``: Name of the base class.
- ``description``: Description of the plugin.

.. code-block:: xml

  <library path="nav2_pure_pursuit_controller">
    <class type="nav2_pure_pursuit_controller::PurePursuitController" base_class_type="nav2_core::Controller">
      <description>
        This is pure pursuit controller
      </description>
    </class>
  </library>

3. Next step would be to export plugin using ``CMakeLists.txt`` by using CMake function ``pluginlib_export_plugin_description_file()``. This function installs the plugin description file to ``share`` directory and sets ament indexes to make it discoverable.

.. code-block:: text

  pluginlib_export_plugin_description_file(nav2_core pure_pursuit_controller_plugin.xml)

4. The plugin description file should also be added to ``package.xml``

.. code-block:: xml

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/pure_pursuit_controller_plugin.xml" />
  </export>

5. Compile, and it should be registered. Next, we'll use this plugin.

3- Pass the plugin name through the params file
-----------------------------------------------

To enable the plugin, we need to modify the ``nav2_params.yaml`` file as below

.. code-block:: text

  controller_server:
    ros__parameters:
      controller_plugins: ["FollowPath"]

      FollowPath:
        plugin: "nav2_pure_pursuit_controller::PurePursuitController"
        debug_trajectory_details: True
        desired_linear_vel: 0.2
        lookahead_dist: 0.4
        max_angular_vel: 1.0
        transform_tolerance: 1.0

In the above snippet, you can observe the mapping of our ``nav2_pure_pursuit_controller/PurePursuitController`` controller to its id ``FollowPath``. 
To pass plugin-specific parameters we have used ``<plugin_id>.<plugin_specific_parameter>``.

4- Run Pure Pursuit Controller plugin
-------------------------------------

Run Turtlebot3 simulation with enabled Nav2. Detailed instructions on how to make it run are written at :ref:`getting_started`. Below is a shortcut command for that:

.. code-block:: bash

  $ ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/your_params_file.yaml

Then goto RViz and click on the "2D Pose Estimate" button at the top and point the location on the map as it was described in :ref:`getting_started`. 
The robot will localize on the map and then click on the "Nav2 goal" and click on the pose where you want your robot to navigate to. 
After that controller will make the robot follow the global path.
