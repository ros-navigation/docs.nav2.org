.. _writing_new_nav2navigator_plugin:

Writing a New Navigator Plugin
******************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

Overview
========

This tutorial shows how to create your own behavior-tree navigator `plugin <https://index.ros.org/p/pluginlib/>`_ based on the ``nav2_core::BehaviorTreeNavigator`` base class.

In this tutorial, we will be reviewing the ``Navigate to Pose`` behavior-tree navigator plugin, which is the foundational navigator of Nav2 and complimentary behavior to ROS 1 Navigation. This completes point-to-point navigation. This tutorial will be reviewing the code and structure as of ROS 2 Iron. While small variations may be made over time, this should be sufficient to get started writing your own navigator if you choose as we do not expect major API changes on this system.

It may be beneficial to write your own Navigator if you have a custom action message definition you'd like to use with Navigation rather than the provided ``NavigateToPose`` or ``NavigateThroughPoses`` interfaces (e.g. doing complete coverage or containing additional constraint information). The role of the Navigators are to extract information from requests to pass to the behavior tree / blackboard, populate feedback and responses, and maintain the state of the behavior tree if relevant. The behavior tree XML will define the actual navigation logic used. 

Requirements
============

- ROS 2 (binary or build-from-source)
- Nav2 (Including dependencies)
- Gazebo
- Turtlebot3

Tutorial Steps
==============

1- Create a new Navigator Plugin
--------------------------------

We will be implementing pure point-to-point navigation behavior. The code in this tutorial can be found in `Nav2's BT Navigator package <https://github.com/ros-planning/navigation2/nav2_bt_navigator>`_ as the ``NavigateToPoseNavigator``. This package can be considered as a reference for writing your own plugin.

Our example plugin class ``nav2_bt_navigator::NavigateToPoseNavigator`` inherits from the base class ``nav2_core::BehaviorTreeNavigator``. The base class provides a set of virtual methods to implement a navigator plugin. These methods are called at runtime by the BT Navigator server or as a response to ROS 2 actions to process a navigation request.

Note that this class has itself a base class of ``NavigatorBase``. This class is to provide a non-templated base-class for use in loading the plugins into vectors for storage and calls for basic state transition in the lifecycle node. Its members (e.g. ``on_XYZ``) are implemented for you in ``BehaviorTreeNavigator`` and marked as ``final`` so they are not possible to be overrided by the user. The API that you will be implementing for your navigator are the virtual methods within ``BehaviorTreeNavigator``, not ``NavigatorBase``. These ``on_XYZ`` APIs are implemented in necessary functions in ``BehaviorTreeNavigator`` to handle boilerplate logic regarding the behavior tree and action server to minimize code duplication across the navigator implementations (e.g. ``on_configure`` will create the action server, register callbacks, populate the blackboard with some necessary basic information, and then call a user-defined ``configure`` function for any additional user-specific needs).

The list of methods, their descriptions, and necessity are presented in the table below:

+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| **Virtual method**        | **Method description**                                                                | **Requires override?** |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| getDefaultBTFilepath()    | Method is called on initialization to retrieve the default BT filepath to use for     | Yes                    |
|                           | navigation. This may be done via parameters, hardcoded logic, sentinal files, etc.    |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| configure()               | Method is called when BT navigator server enters on_configure state. This method      | No                     |
|                           | should implement operations which are neccessary before navigator goes to an active   |                        |
|                           | state, such as getting parameters, setting up the blackboard, etc.                    |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| activate()                | Method is called when BT navigator server enters on_activate state. This method       | No                     |
|                           | should implement operations which are neccessary before navigator goes to an active   |                        |
|                           | state, such as create clients and subscriptions.                                      |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| deactivate()              | Method is called when BT navigator server enters on_deactivate state.  This           | No                     |
|                           | method should implement operations which are neccessary before navigator goes to an   |                        |
|                           | inactive state.                                                                       |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| cleanup()                 | Method is called when BT navigator server goes to on_cleanup state. This method       | No                     |
|                           | should clean up resources which are created for the navigator.                        |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| goalReceived()            | Method is called when a new goal is received by the action server to process. It may  | Yes                    |
|                           | accept or deny this goal with its return signature. If accepted, it may need to load  |                        |
|                           | the appropriate parameters from the request (e.g. which BT to use), add request       |                        |
|                           | parameters to the blackboard for your applications use, or reset internal state.      |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| onLoop()                  | Method is called periodically while the behavior tree is looping to check statuses    | Yes                    |
|                           | or more commonly to publish action feedback statuses to the client.                   |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| onPreempt()               | Method is called when a new goal is requesting preemption over the existing           | Yes                    |
|                           | goal currently being processed. If the new goal is viable, it should make all         |                        |
|                           | appropriate updates to the BT and blackboard such that this new request may           |                        |
|                           | immediately start being processed without hard cancelation of the initial task        |                        |
|                           | (e.g. preemption).                                                                    |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| goalCompleted()           | Method is called when a goal is completed to populate the action result object or     | Yes                    |
|                           | do any additional checks required at the end of a task.                               |                        |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+
| getName()                 | Method is called to get the name of this navigator type                               | Yes                    |
+---------------------------+---------------------------------------------------------------------------------------+------------------------+

In the Navigate to Pose Navigator, ``configure()`` method must determine the blackboard parameter names where the goal and paths are being stored, as these are key values for processing feedback in ``onLoop`` and for the different behavior tree nodes to communicate this information between themselves. Additionally and uniquely to this navigator type, we also create a client to itself and a subscription to the ``goal_pose`` topic such that requests from the default configurations of Rviz2 using the *Goal Pose* tool will be processed.

.. code-block:: c++

    bool NavigateToPoseNavigator::configure(
      rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
      std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
    {
      start_time_ = rclcpp::Time(0);
      auto node = parent_node.lock();

      if (!node->has_parameter("goal_blackboard_id")) {
        node->declare_parameter("goal_blackboard_id", std::string("goal"));
      }

      goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();

      if (!node->has_parameter("path_blackboard_id")) {
        node->declare_parameter("path_blackboard_id", std::string("path"));
      }

      path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

      // Odometry smoother object for getting current speed
      odom_smoother_ = odom_smoother;

      self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

      goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&NavigateToPoseNavigator::onGoalPoseReceived, this, std::placeholders::_1));
      return true;
    }

The values of the blackboard IDs are stored alongside the odometry smoother the BT Navigator provides for populating meaningful feedback later. Complimentary to this, the ``cleanup`` method will reset these resources. The activate and deactivate methods are not used in this particular navigator.

.. code-block:: c++

    bool NavigateToPoseNavigator::cleanup()
    {
      goal_sub_.reset();
      self_client_.reset();
      return true;
    }

In the ``getDefaultBTFilepath()``, we use a parameter ``default_nav_to_pose_bt_xml`` to get the default behavior tree XML file to use if none is provided by the navigation request and to initialize the BT Navigator with a behavior tree hot-loaded. If one is not provided in the parameter files, then we grab a known and reasonable default XML file in the ``nav2_bt_navigator`` package: 

.. code-block:: c++

    std::string NavigateToPoseNavigator::getDefaultBTFilepath(
      rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
    {
      std::string default_bt_xml_filename;
      auto node = parent_node.lock();

      if (!node->has_parameter("default_nav_to_pose_bt_xml")) {
        std::string pkg_share_dir =
          ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
        node->declare_parameter<std::string>(
          "default_nav_to_pose_bt_xml",
          pkg_share_dir +
          "/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml");
      }

      node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filename);

      return default_bt_xml_filename;
    }

When a goal is received, we need to determine if this goal is valid and should be processed.
The ``goalReceived`` method provides you the ``goal`` and a return value if it is being processed or not. This information is sent back to the action server to notify the client. In this case, we want to make sure that the goal's behavior tree is valid or else we cannot proceed. If it is valid, then we can initialize the goal pose onto the blackboard and reset some state in order to cleanly process this new request.

.. code-block:: c++

    bool NavigateToPoseNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
    {
      auto bt_xml_filename = goal->behavior_tree;

      if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
        RCLCPP_ERROR(
          logger_, "BT file not found: %s. Navigation canceled.",
          bt_xml_filename.c_str());
        return false;
      }

      initializeGoalPose(goal);

      return true;
    }

Once this goal is completed, we need to populate the Action's result, if required and meaningful. In this navigator's case, it contains no result information when the navigation request was completed successfully, so this method is empty. For other navigator types, you may populate the ``result`` object provided.

.. code-block:: c++

    void NavigateToPoseNavigator::goalCompleted(
      typename ActionT::Result::SharedPtr /*result*/,
      const nav2_behavior_tree::BtStatus /*final_bt_status*/)
    {
    }

If however a goal is preempted (e.g. a new action request comes in while an existing request is being processed), the ``onPreempt()`` method is called to determine if the new request is genuine and appropriate to preempt the currently processing goal. For example, it might not be wise to accept a preeemption request if that request is fundamentally different in nature from an existing behavior tree task or when your existing task is of a higher priority.

.. code-block:: c++

    void NavigateToPoseNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
    {
      RCLCPP_INFO(logger_, "Received goal preemption request");

      if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
        (goal->behavior_tree.empty() &&
        bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
      {
        // if pending goal requests the same BT as the current goal, accept the pending goal
        // if pending goal has an empty behavior_tree field, it requests the default BT file
        // accept the pending goal if the current goal is running the default BT file
        initializeGoalPose(bt_action_server_->acceptPendingGoal());
      } else {
        RCLCPP_WARN(
          logger_,
          "Preemption request was rejected since the requested BT XML file is not the same "
          "as the one that the current goal is executing. Preemption with a new BT is invalid "
          "since it would require cancellation of the previous goal instead of true preemption."
          "\nCancel the current goal and send a new action request if you want to use a "
          "different BT XML file. For now, continuing to track the last goal until completion.");
        bt_action_server_->terminatePendingGoal();
      }
    }

Note that here you can also see the ``initializeGoalPose`` method called. This method will set the goal parameters for this navigator onto the blackboard and reset important state information to cleanly re-use a behavior tree without old state information, as shown below:

.. code-block:: c++

    void
    NavigateToPoseNavigator::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal)
    {
      RCLCPP_INFO(
        logger_, "Begin navigating from current location to (%.2f, %.2f)",
        goal->pose.pose.position.x, goal->pose.pose.position.y);

      // Reset state for new action feedback
      start_time_ = clock_->now();
      auto blackboard = bt_action_server_->getBlackboard();
      blackboard->set<int>("number_recoveries", 0);  // NOLINT

      // Update the goal pose on the blackboard
      blackboard->set<geometry_msgs::msg::PoseStamped>(goal_blackboard_id_, goal->pose);
    }

The recovery counter and start time are both important feedback terms for a client to understand the state of the current task (e.g. if its failing, having problems, or taking exceptionally long). The setting of the goal on the blackboard is taken by the ``ComputePathToPose`` BT Action node to plan a new route to the goal (and then who's path is communicated to the ``FollowPath`` BT node via the blackboard ID previously set).

The final function implemented is ``onLoop``, which is simplified below for tutorial purposes. While anything can be done in this method, which is called as the BT is looping through the tree, it is common to use this as an opportunity to populate any necessary feedback about the state of the navigation request, robot, or metadata that a client might be interested in.

.. code-block:: c++

    void NavigateToPoseNavigator::onLoop()
    {
      auto feedback_msg = std::make_shared<ActionT::Feedback>();

      geometry_msgs::msg::PoseStamped current_pose = ...;
      auto blackboard = bt_action_server_->getBlackboard();
      nav_msgs::msg::Path current_path;
      blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, current_path);

      ...

      feedback_msg->distance_remaining = distance_remaining;
      feedback_msg->estimated_time_remaining = estimated_time_remaining;

      int recovery_count = 0;
      blackboard->get<int>("number_recoveries", recovery_count);
      feedback_msg->number_of_recoveries = recovery_count;
      feedback_msg->current_pose = current_pose;
      feedback_msg->navigation_time = clock_->now() - start_time_;

      bt_action_server_->publishFeedback(feedback_msg);
    }

2- Exporting the navigator plugin
---------------------------------

Now that we have created our custom navigator, we need to export our plugin so that it would be visible to the BT Navigator server. 
Plugins are loaded at runtime, and if they are not visible, then our server won't be able to load it. In ROS 2, exporting and loading 
plugins is handled by ``pluginlib``.

Coming to our tutorial, class ``nav2_bt_navigator::NavigateToPoseNavigator`` is loaded dynamically as ``nav2_core::NavigatorBase`` which is our base class due to the subtleties previously described.

1. To export the controller, we need to provide two lines

.. code-block:: c++
 
 #include "pluginlib/class_list_macros.hpp"
 PLUGINLIB_EXPORT_CLASS(nav2_bt_navigator::NavigateToPoseNavigator, nav2_core::NavigatorBase)

Note that it requires pluginlib to export out the plugin's class. Pluginlib would provide as macro ``PLUGINLIB_EXPORT_CLASS``, which does all the work of exporting.

It is good practice to place these lines at the end of the file, but technically, you can also write at the top.

2. The next step would be to create the plugin's description file in the root directory of the package. For example, ``navigator_plugin.xml`` file in our tutorial package. This file contains the following information

- ``library path``: Plugin's library name and it's location.
- ``class name``: Name of the class.
- ``class type``: Type of class.
- ``base class``: Name of the base class.
- ``description``: Description of the plugin.

.. code-block:: xml

  <library path="nav2_bt_navigator">
    <class type="nav2_bt_navigator::NavigateToPoseNavigator" base_class_type="nav2_core::NavigatorBase">
      <description>
        This is pure point-to-point navigation
      </description>
    </class>
  </library>

3. Next step would be to export plugin using ``CMakeLists.txt`` by using CMake function ``pluginlib_export_plugin_description_file()``. This function installs the plugin description file to ``share`` directory and sets ament indexes to make it discoverable.

.. code-block:: text

  pluginlib_export_plugin_description_file(nav2_core navigator_plugin.xml)

4. The plugin description file should also be added to ``package.xml``

.. code-block:: xml

  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/navigator_plugin.xml" />
  </export>

5. Compile, and it should be registered. Next, we'll use this plugin.

3- Pass the plugin name through the params file
-----------------------------------------------

To enable the plugin, we need to modify the ``nav2_params.yaml`` file as below

.. code-block:: text

    bt_navigator:
      ros__parameters:
        use_sim_time: true
        global_frame: map
        robot_base_frame: base_link
        transform_tolerance: 0.1
        default_nav_to_pose_bt_xml: replace/with/path/to/bt.xml
        default_nav_through_poses_bt_xml: replace/with/path/to/bt.xml
        goal_blackboard_id: goal
        goals_blackboard_id: goals
        path_blackboard_id: path
        navigators: ['navigate_to_pose', 'navigate_through_poses']
        navigate_to_pose:
          plugin: "nav2_bt_navigator/NavigateToPoseNavigator"
        navigate_through_poses:
          plugin: "nav2_bt_navigator/NavigateThroughPosesNavigator"


In the above snippet, you can observe the mapping of our ``nav2_bt_navigator/NavigateToPoseNavigator`` plugin to its id ``navigate_to_pose``. 
To pass plugin-specific parameters we have used ``<plugin_id>.<plugin_specific_parameter>``.

4- Run plugin
-------------

Run Turtlebot3 simulation with enabled Nav2. Detailed instructions on how to make it run are written at :ref:`getting_started`. Below is a shortcut command for that:

.. code-block:: bash

  $ ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/your_params_file.yaml

Then goto RViz and click on the "2D Pose Estimate" button at the top and point the location on the map as it was described in :ref:`getting_started`. 
The robot will localize on the map and then click on the "Nav2 goal" and click on the pose where you want your robot to navigate to. 
After that navigator will take over with the behavior tree XML file behavior definition provided to it.
