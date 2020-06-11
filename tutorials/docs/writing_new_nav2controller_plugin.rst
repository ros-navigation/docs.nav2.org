.. _writing_new_nav2controller_plugin:

Writing a New Controller Plugin
*******************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

Overview
========

This tutorial shows how to create you own controller `plugin <https://index.ros.org/p/pluginlib/>`_.

In this tutorial we will be implementing the pure pursuit path tracking algorithm, based on the `paper <https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf>`_. 
It is recommended you go through it.

Requirements
============

- ROS2 (binary or build-from-source)
- Navigation2 (Including dependencies)
- Gazebo
- Turtlebot3

Tutorial Steps
==============

1- Create a new Controller Plugin
---------------------------------

We will be implementing the pure pursuit controller. The annotated code in this tutorial can be found in `navigation_tutorials <https://github.com/ros-planning/navigation2_tutorials>`_ repository as the ``nav2_pure_pursuit_controller``
This package can be a considered as a reference for writing controller plugin.

Our plugin class ``nav2_pure_pursuit_controller::PurePursuitController`` inherits from the base class ``nav2_core::Controller``. The base class provides a
set of virtual methods to implement a controller plugin. These methods are called at runtime by the controller server to compute velocity commands.
The list of methods and their descriptions and necessity are presented in the table below:

+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| **Virtual method**        | **Method description**                                                                | **Requires override?** |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| configure()               | This methods should perform declarations of ROS parameters and                        | Yes                    |
|                           | initialization of controller's member variables i.e. tasks that need to be            |                        |
|                           | performed once in the lifetime of the controller. This method passes 4                |                        |
|                           | parameters: shared pointer to parent node, controller name, tf buffer pointer         |                        |
|                           | and shared pointer to costmap.                                                        |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| activate()                | This method should implement operations which are neccessary before the               | Yes                    |
|                           | controller goes to an active state.                                                   |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| deactivate()              | This method should implement operations which are neccessary before the               | Yes                    |
|                           | controller goes to the inactive state.                                                |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| cleanup()                 | This method should clean up resoures which are created for the controller.            | Yes                    |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| setPlan()                 | This method is called when the global plan is updated. This method passes 1           | Yes                    |
|                           | parameter: a reference to the global plan.                                            |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| computeVelocityCommands() | This method is called when a new velocity command is required by the robot to follow  | Yes                    |
|                           | to follow the global path. This method returns a `geometry_msgs::msg::TwistStamped`   |                        |
|                           | which represents the velocity command for the robot to drive.  This method passes     |                        |
|                           | 2 parameters: reference to the current robot pose and its current velocity.           |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| isGoalReached()           | This method is called to check whether the robot has reached the goal or not.         | Yes                    |
|                           | takes in 2 parameters: reference to the current robot pose and its current velocity,  |                        |
|                           | and returns a boolean                                                                 |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+

In this tutorial, we will have used the following methods:

1. ``PurePursuitController::configure()`` 

In controllers, this method is used to set member variables, declare ROS parameters with default parameters and read their values.

.. code-block:: c++

  node_ = parent;
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

The passed in arguments are stored in member variables so that hey can be used at a later stage if needed.

.. code-block:: c++

  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + ".desired_linear_vel",rclcpp::ParameterValue(0.2));

``declare_parameter_if_not_declared`` is used to declare a ROS parameters for that node (if it hasn't been declared previously) using the specified name and sets the default value.
Here, we prepend the mapped plugin name to the parameter like this ``plugin_name_ + ".desired_linear_vel"``. This is done as Navigation2 allows loading multiple 
controllers, so to prevent parameter name collisions across different controllers we refer to our parameters in the following format ``<mapped_name_of_plugin>.<name_of_parameter>``.

.. code-block:: c++

  node_->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);

This is used to fetch the specified parameter ``plugin_name_ + "desired_linear_vel"`` and store the value in the member variable ``desired_linear_vel_``.

Eg: If our controller plugin name is mapped to ``FollowPath`` then our parameter will be declared as ``FollowPath.desired_linear_vel``, therefore ``FollowPath``
acts as a namespace for our plugin specific parameters.

2. ``PurePursuitController::setPlan()`` 

This method is used to store the global plan and the goal pose (i.e. last pose of the global plan). Additionaly it is used to transform the global plan into 
the required frame.

.. code-block:: c++

  nav_2d_msgs::msg::Path2D path2d = nav_2d_utils::pathToPath2D(path);

This is used to convert the global path from ``nav_msgs::msg::Path`` to ``nav_2d_msgs::msg::Path2D``. This is done as the ``Path2D`` message is easier to
work with for 2D planning.

.. code-block:: c++

  global_plan_ = transformGlobalPlan(path2d); 

The global path needs to be transformed into the appropriate frame before using it. This is generally same as costmap frame.
In our case, this frame is the robot's own frame. So, we transform the global path into the robot's frame.

.. code-block:: c++

  goal_pose_ = path.poses.back();

We store the goal pose i.e. last pose of the global path, for later checking if we have reached the goal or not.

3. ``PurePursuitController::computeVelocityCommands()`` 
   This method is used to calculate the best command given the current pose and velocity. The pure pursuit algorithm gives velocity commands such that the robot
   tries to follow the global path as closely as possible. This algorithm assumes a constant linear velocity and computes the angular velocity based on the curvature of the global path.

.. code-block:: c++

  auto goal_pose = std::find_if(
  global_plan_.poses.begin(), global_plan_.poses.end(),
  [&](const auto & global_plan_pose) {
    return hypot(global_plan_pose.x, global_plan_pose.y) >= lookahead_dist_;
  });

This is used to find the closest point on the path which is further than the lookahead distance.

.. code-block:: c++

  auto curvature = 2.0 * goal_pose->y / (goal_pose->x * goal_pose->x + goal_pose->y * goal_pose->y);
  auto angular_vel = desired_linear_vel_ * curvature;

Using the closest point on the path, computed earlier, the curvature of the path is determined and using that the angular velocity is calculated.

.. code-block:: c++

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.twist.linear.x = desired_linear_vel_;
  cmd_vel.twist.angular.z = max(-1.0 * abs(max_angular_vel_), min(angular_vel, abs(max_angular_vel_)));

  return cmd_vel;

A new TwistStamped message is created to store the computed velocity and then this message is returned.


4. ``PurePursuitController::isGoalReached()`` 

This method is used to check whether the robot has reached the goal pose or not based on the current pose and velocity.

.. code-block:: c++

  return hypot(
    pose.pose.position.x - goal_pose_.pose.position.x,
    pose.pose.position.y - goal_pose_.pose.position.y) <= goal_tolerance_;

Here, we check whether the current robot pose, is within a certain distance (i.e. goal tolerance) from from the goal pose. If it is then we return true
indicating that the robot has reached the goal, else we return false.


The remaining methods are not used but its mandatory to override them. As per the rules, we did override all but left them empty.

2- Exporting the controller plugin
----------------------------------

Now that we have created our custom controller, we need to export our controller plugin so that it would be visible to the controller server. Plugins are loaded at runtime and if they are not visible, then our controller server won't be able to load it. In ROS2, exporting and loading plugins is handled by ``pluginlib``.

Coming to our tutorial, class ``nav2_pure_pursuit_controller::PurePursuitController`` is loaded dynamically as ``nav2_core::Controller`` which is our base class.

1. To export the controller, we need to provide two lines

.. code-block:: c++
  
  #include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)

Note that it requires pluginlib to export out plugin's class. Pluginlib would provide as macro ``PLUGINLIB_EXPORT_CLASS`` which does all the work of exporting.

It is good practice to place these lines at the end of the file but technically, you can also write at the top.

2. Next step would be to create plugin's description file in the root directory of the package. For example, ``pure_pursuit_controller_plugin.xml`` file in our tutorial package. This file contains following information

 - ``library path``: Plugin's library name and it's location.
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

3. Next step would be to export plugin using ``CMakeLists.txt`` by using cmake function ``pluginlib_export_plugin_description_file()``. This function installs plugin description file to ``share`` directory and sets ament indexes to make it discoverable.

.. code-block:: text

  pluginlib_export_plugin_description_file(nav2_core pure_pursuit_controller_plugin.xml)

4. Plugin description file should also be added to ``package.xml``

.. code-block:: xml

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/pure_pursuit_controller_plugin.xml" />
  </export>

5. Compile and it should be registered. Next, we'll use this plugin.

3- Pass the plugin name through params file
-------------------------------------------

To enable the plugin, we need to modify the ``nav2_params.yaml`` file as below

replace following params

.. code-block:: text

  controller_server:
  ros__parameters:
    controller_plugin_types: ["dwb_core::DWBLocalPlanner"]
    controller_plugin_ids: ["FollowPath"]

with

.. code-block:: text

  controller_server:
  ros__parameters:
    controller_plugin_types: ["nav2_pure_pursuit_controller/PurePursuitController"]
    controller_plugin_ids: ["FollowPath"]

In the above snippet, you can observe the mapping of our ``nav2_pure_pursuit_controller/PurePursuitController`` controller to its id ``FollowPath``. To pass plugin-specific parameters we have used ``<plugin_id>.<plugin_specific_parameter>``.

4- Run Pure Pursuit Controller plugin
-------------------------------------

Run Turtlebot3 simulation with enabled navigation2. Detailed instruction how to make it are written at :ref:`getting_started`. Below is shortcut command for that:

.. code-block:: bash

  $ ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/your_params_file.yaml

Then goto RViz and click on the "2D Pose Estimate" button at the top and point the location on map as it was described in :ref:`getting_started`. Robot will localize on the map and then click on "Navigation2 goal" and click on the pose where you want your robot to navigate to. After that controller will make the robot to follow the global path.
