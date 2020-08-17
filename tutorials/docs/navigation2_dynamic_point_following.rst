.. _navigation2-dynamic-point-following:

Dynamic Object Following
***********************

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

This tutorial shows how to use Navigation2 for a different task of going from point A to point B. In this case, we will use Navigation2 to follow a moving object at a distance indefinitely.

This task is useful in cases such as following a person, as shown in the next video of "Carry My Luggage" RoboCup @ Home test, in which the `CATIE Robotics <https://robotics.catie.fr/>`_ team performs the test successfully, or this other video 
of a real (future) world application:

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

- The changes to achieve this functionality are limited to the behavior tree used to navigate. This behavior tree can be selected in the NavigateToPose action when this functionality is required, or it can be the default behavior tree.
- The configuration of the planner and the controller will not be modified.
- The action will indefinitely run until it is canceled by who initiated it.

The detection of the dynamic object (person) pose to follow is outside the scope of this tutorial. As shown in the following diagram, your application should provide a detector for the object(s) of interest, 
send the initial pose to the ``NavigateToPose`` action, and update it on a topic for the duration of the task:


.. image:: images/navigation2_dynamic_point_following/main_diagram.png
    :width: 48%


Before completing this tutorial, please look at the previous two tutorials on navigation in simulation and physical hardware.
This tutorial assumes knowledge of navigation and basic understanding of behavior trees.

Tutorial Steps
==============

0- Create the Behavior Tree
---------------------------

Let's start from this simple behavior tree. This behavior tree with replan a new path every 1hz and pass that path to the controller to follow:

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

First of all, let's make that this behavior runs while there is not any failure. For this purpose, we will use the ``KeepRunningUntilFailure`` control node.

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

We will use the decorator ``GoalUpdater`` to update the dynamic object pose to follow. This node takes as input the current goal and subscribes to the topic ``/goal_update``. It set the new goal as ``updated_goal``:

- The original goal, if nothing received in ``/goal_update``.
- Or, the pose received in ``/goal_update``.

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

Let's save this behavior tree and use it in our navigation task.

1- Setup Rviz clicked point
---------------------------

In this tutorial, we are going to use RViz instead of a full application. We will use the "clicked point" button on the toolbar to send goal updates to Navigation2. This button allows you to 
publish coordinates in the topic ``/clicked_point``. This point needs to be sent to the behavior tree, using the program ``clicked_point_to_pose``, from `this repo <https://github.com/fmrico/nav2_test_utils>`_. Clone 
this repo in your workspace, build, and type in a terminal.

``ros2 run nav2_test_utils clicked_point_to_pose``

2- Run Dynamic Object Following in Navigation2 Simulation
---------------------------------------------------------

Start Navigation2 in one terminal:

``ros2 launch nav2_bringup tb3_simulation_launch.py default_bt_xml_filename:=/path/to/bt.xml``

Open RViz and, after initialize the robot position, command the robot to navigate to any position. Use the button clicked point to change the goal, as shown in the video in the head of this tutorial.

.. raw:: html

    <h1 align="center">
      <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700" height="450" src="https://www.youtube.com/embed/r4fIkcktZUM?autoplay=1" frameborder="1" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
      </div>
    </h1>
