.. _navigation2-dynamic-point-following:

Dynamic Object Following
************************

- `Overview`_
- `Tutorial Steps`_

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700" height="450" src="https://www.youtube.com/embed/sRodzrrJChA?autoplay=1" frameborder="1" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
      </div>
    </h1>

Overview
========

This tutorial shows how to use Nav2 for a different task other than going from point A to point B. In this case, we will use Nav2 to follow a moving object at a distance indefinitely.

This task is useful in cases such as following a person or another robot. Below are some sample videos of applications that could be created using this capability. The "Carry My Luggage" RoboCup @ Home test, in which the `CATIE Robotics <https://robotics.catie.fr/>`_ team performs the test successfully and this real (future) world application:

.. raw:: html

    <h1 align="center">
      <div>
        <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
          <iframe width="450" height="300" src="https://www.youtube.com/embed/lTjKO4M7yZc?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
          <iframe width="450" height="300" src="https://www.youtube.com/embed/KgRKyzsja9Q?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
        </div>
      </div>
    </h1>

The requirements for this task are as follows:

- Changes are limited to the behavior tree used to navigate. This behavior tree can be selected in the ``NavigateToPose`` action when required, or it can be the default behavior tree. It is made up of run-time configurable plugins.
- The configuration of the planner and the controller will not be modified.
- The action will indefinitely run until it is canceled by who initiated it.

The detection of the dynamic object (like a person) to follow is outside the scope of this tutorial. As shown in the following diagram, your application should provide a detector for the object(s) of interest, 
send the initial pose to the ``NavigateToPose`` action, and update it on a topic for the duration of the task. Many different types of detectors exist that you can leverage for this application:


.. image:: images/navigation2_dynamic_point_following/main_diagram.png
    :width: 48%

Tutorial Steps
==============

0- Create the Behavior Tree
---------------------------

Let's start from this simple behavior tree. This behavior tree replans a new path at 1 hz and passes that path to the controller to follow:

.. code-block:: xml

  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
    </BehaviorTree>
  </root>

First, let's make this behavior run until there's a failure. For this purpose, we will use the ``KeepRunningUntilFailure`` control node.

.. code-block:: xml

  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <KeepRunningUntilFailure>
          <FollowPath path="{path}" controller_id="FollowPath"/>
        </KeepRunningUntilFailure>
      </PipelineSequence>
    </BehaviorTree>
  </root>

We will then use the decorator ``GoalUpdater`` to accept updates of the dynamic object pose we're trying to follow. This node takes as input the current goal and subscribes to the topic ``/goal_update``. It sets the new goal as ``updated_goal`` if a new goal on that topic is received.

.. code-block:: xml

  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">
            <ComputePathToPose goal="{updated_goal}" path="{path}" planner_id="GridBased"/>
          </GoalUpdater>
        </RateController>
        <KeepRunningUntilFailure>
          <FollowPath path="{path}" controller_id="FollowPath"/>
        </KeepRunningUntilFailure>
      </PipelineSequence>
    </BehaviorTree>
  </root>

To stay at a certain distance from the target, we will use the action node ``TruncatePath``. This node modifies a path making it shorter so we don't try to navigate into the object of interest. We can set up the desired distance to the goal using the input port ``distance``.

.. code-block:: xml

  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <Sequence>
            <GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">
              <ComputePathToPose goal="{updated_goal}" path="{path}" planner_id="GridBased"/>
            </GoalUpdater>
           <TruncatePath distance="1.0" input_path="{path}" output_path="{truncated_path}"/>
          </Sequence>
        </RateController>
        <KeepRunningUntilFailure>
          <FollowPath path="{truncated_path}" controller_id="FollowPath"/>
        </KeepRunningUntilFailure>
      </PipelineSequence>
    </BehaviorTree>
  </root>

Now, you may save this behavior tree and use it in our navigation task.

For reference, this exact behavior tree is `made available <https://github.com/ros-planning/navigation2/blob/main/nav2_bt_navigator/behavior_trees/follow_point.xml>`_ to you batteries included in the ``nav2_bt_navigator`` package.

1- Setup Rviz clicked point
---------------------------

We are going to use RViz instead of a full application so you can test at home without finding a detector to get started. We will use the "clicked point" button on the toolbar to substitute object detections to provide goal updates to Nav2. This button allows you to
publish coordinates in the topic ``/clicked_point``. This point needs to be sent to the behavior tree, using the program ``clicked_point_to_pose``, from `this repo <https://github.com/fmrico/nav2_test_utils>`_. Clone 
this repo in your workspace, build, and type in a terminal.

``ros2 run nav2_test_utils clicked_point_to_pose``

Optionally, you can remap this topic in your rviz configuration file to ``goal_updates``.

2- Run Dynamic Object Following in Nav2 Simulation
--------------------------------------------------

Start Nav2 in one terminal:

``ros2 launch nav2_bringup tb3_simulation_launch.py default_bt_xml_filename:=/path/to/bt.xml``

Open RViz and, after initialize the robot position, command the robot to navigate to any position. Use the button clicked point to simulate a new detection of the object of interest, as shown in the video in the head of this tutorial.

When you have a detector detecting your obstacle at a higher rate (1 hz, 10 hz, 100 hz) you will see a far more reactive robot following your detected object of interest!

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700" height="450" src="https://www.youtube.com/embed/r4fIkcktZUM?autoplay=1" frameborder="1" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
      </div>
    </h1>
