.. _custom_behavior_tree:

Creating a Custom Behavior Tree in ROS2 / Nav2
**********************************************

- `Overview`_
- `Prerequisites`_
- `Navigate With Replanning and Recovery`_
- `NavigateWithReplanning`_
- `RecoveryFallback`_
- `Custom Action`_
- `Adding to Launch File`_
- `Testing`_

Overview
========

This document serves as a reference guide to the main behavior tree (BT) used in Nav2,
and explains the process for developing and customizing this BT for ROS 2 and Nav2.

There are many example behavior trees provided in ``nav2_bt_navigator/behavior_trees``,
but these usually have to be configured and modified based on the application of the robot. 
The following tutorial will walk through the current main default BT ``navigate_w_replanning_and_recovery.xml``
and will show users how to modify this BT in potentially useful ways, using the example of developing a BT that follows a predefined path.

Prerequisites
=============

* Have a valid installation of Nav2 
* Have a robot (simulated, or physical) that can be used for testing that already works with Nav2
* Become familiar with the concept of a behavior tree before continuing with this tutorial
    * There is a short explaination in `navigation concepts <../../concepts/index.html>`_
    * General tutorial (not Nav2 specific) on the `BehaviorTree CPP V3 <https://www.behaviortree.dev/>`_ website.

Navigate With Replanning and Recovery
=====================================

The following section will describe in detail the concept of the main and default BT currently used in Nav2, ``navigate_w_replanning_and_recovery.xml``.
This behavior tree replans the global path periodically at 1 Hz and it also has recovery actions.

.. code-block:: xml

    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
          <RecoveryNode number_of_retries="6" name="NavigateRecovery">
          <PipelineSequence name="NavigateWithReplanning">
              <RateController hz="1.0">
              <RecoveryNode number_of_retries="1" name="ComputePathToPose">
                  <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                  <ReactiveFallback name="ComputePathToPoseRecoveryFallback">
                  <GoalUpdated/>
                  <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
                  </ReactiveFallback>
              </RecoveryNode>
              </RateController>
              <RecoveryNode number_of_retries="1" name="FollowPath">
              <FollowPath path="{path}" controller_id="FollowPath"/>
              <ReactiveFallback name="FollowPathRecoveryFallback">
                  <GoalUpdated/>
                  <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
              </ReactiveFallback>
              </RecoveryNode>
          </PipelineSequence>
          <ReactiveFallback name="RecoveryFallback">
              <GoalUpdated/>
              <RoundRobin name="RecoveryActions">
              <Sequence name="ClearingActions">
                  <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
                  <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
              </Sequence>
              <Spin spin_dist="1.57"/>
              <Wait wait_duration="5"/>
              <BackUp backup_dist="0.15" backup_speed="0.025"/>
              </RoundRobin>
          </ReactiveFallback>
          </RecoveryNode>
      </BehaviorTree>
    </root>

The above XML probably looks complex and overwhelming, but we can represent this behavior tree as an actual tree
using this `XML ASCII tool <https://nickpisacane.github.io/AsciiTree/>`_.

.. code-block::

                                                                                                   root                                                                                               
                                                                                                     |                                                                                                
                                                                                                    _|                                                                                                
                                                                                                    |                                                                                                 
                                                                                                    |                                                                                                 
                                                                                              BehaviorTree                                                                                            
                                                                                                    |                                                                                                 
                                                                                                   _|                                                                                                 
                                                                                                   |                                                                                                  
                                                                                                   |                                                                                                  
                                                                                             RecoveryNode                                                                                             
                                                                                                   |                                                                                                  
                                                          _________________________________________|_____________________________________________________                                             
                                                          |                                                                                             |                                             
                                                          |                                                                                             |                                             
                                                  PipelineSequence                                                                              ReactiveFallback                                      
                                                          |                                                                                             |                                             
                                 _________________________|____________________________                                _________________________________|______                                       
                                 |                                                    |                                |                                      |                                       
                                 |                                                    |                                |                                      |                                       
                          RateController                                        RecoveryNode                      GoalUpdated                            RoundRobin                                   
                                 |                                                    |                                                                       |                                       
                                _|                                  __________________|_____                                                       ___________|__________________________             
                                |                                   |                      |                                                       |                       |     |      |             
                                |                                   |                      |                                                       |                       |     |      |             
                          RecoveryNode                         FollowPath          ReactiveFallback                                            Sequence                  Spin  Wait  BackUp           
                                |                                                          |                                                       |                                                  
             ___________________|________                                       ___________|______                                      ___________|_________                                         
             |                          |                                       |                |                                      |                   |                                         
             |                          |                                       |                |                                      |                   |                                         
     ComputePathToPose          ReactiveFallback                           GoalUpdated  ClearEntireCostmap                     ClearEntireCostmap  ClearEntireCostmap                                 
                                        |                                                                                                                                                             
                             ___________|______                                                                                                                                                       
                             |                |                                                                                                                                                       
                             |                |                                                                                                                                                       
                        GoalUpdated  ClearEntireCostmap                                                                                                                                               

This is likely still a bit overwhelming, but this tree can be broken into two smaller subtrees that we can focus on one at a time.
These smaller subtrees are the children of the top-most ``RecoveryNode``, let's call these the ``NavigateWithReplanning`` subtree and the ``RecoveryFallback`` subtree.
This can be represented in the following way:

.. code-block::

                      root                      
                        |                       
                       _|                       
                       |                        
                       |                        
                 BehaviorTree                   
                       |                        
                      _|                        
                      |                         
                      |                         
                RecoveryNode                    
                      |                         
            __________|___________              
            |                    |              
            |                    |              
 NavigateWithReplanning  RecoveryFallback       

**Warning**
Vocabulary can be a large point of confusion here for a beginner.

* A ``Node`` when discussing BT is entirely diferent than a ``Node`` in the ROS2 context. 
* A ``Recovery`` in the context of BT is entirely different than a ``Recovery`` in the Nav2 context

In Nav2, a ``Recovery`` refers to a specific action executed by the robot. When calling out the ``RecoveryFallback``,
we mean it in the BT context, but when calling out the ``RecoveryFallback`` we mean it in the Nav2 context.

The ``RecoveryNode`` is the parent to these two subtrees, which means, that if the ``NavigateWithReplanning`` subtree returns ``FAILURE``,
the ``RecoveryFallback`` subtree will be ticked. 
* If the ``RecoveryFallback`` subtree then returns ``SUCCESS`` then ``NavigateWithReplanning`` will be executed again.
* Otherwise, if the ``RecoveryFallback`` returns ``FAILURE`` (this is not likely ... more on that later), then the overall tree will try again as determined by the parameter ``number_of_retries``.
* If the ``number_of_retries`` is exceeded, the overall tree will return ``FAILURE``.

The default ``navigate_w_replanning_and_recovery`` has a ``number_of_retries`` of 6, but this parameter should be changed if your use case has more or less acceptable retries.

.. code-block:: xml

    <RecoveryNode number_of_retries="6" name="NavigateRecovery">

For more details regarding the ``RecoveryNode`` please see the `configuration guide <../../configuration/packages/bt-plugins/controls/RecoveryNode.html>`_.

Note that the ``RecoveryNode`` is a custom ``control`` type node made for Nav2, but can be replaced by any other control type node based on the application. 
Replacements in the BT goes without saying for any node, and from here on out I will only call this out for particularly interesting subsitutions.

NavigateWithReplanning
======================

Now that we have gone over the control flow between ``NavigateWithReplanning`` and ``RecoveryFallback``, 
let's focus on ``NavigateWithReplanning``, the main navigation subtree where we hope your robot will spend most of it's time. 

The XML of this subtree is as follows:

.. code-block:: xml

    <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
        <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ReactiveFallback name="ComputePathToPoseRecoveryFallback">
            <GoalUpdated/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            </ReactiveFallback>
        </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
        <FollowPath path="{path}" controller_id="FollowPath"/>
        <ReactiveFallback name="FollowPathRecoveryFallback">
            <GoalUpdated/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </ReactiveFallback>
        </RecoveryNode>
    </PipelineSequence>

And the ASCII representation:

.. code-block::

                                              PipelineSequence                                               
                                                      |                                                      
                             _________________________|____________________________                          
                             |                                                    |                          
                             |                                                    |                          
                      RateController                                        RecoveryNode                     
                             |                                                    |                          
                            _|                                  __________________|_____                     
                            |                                   |                      |                     
                            |                                   |                      |                     
                      RecoveryNode                         FollowPath          ReactiveFallback              
                            |                                                          |                     
         ___________________|________                                       ___________|______               
         |                          |                                       |                |               
         |                          |                                       |                |               
 ComputePathToPose          ReactiveFallback                           GoalUpdated  ClearEntireCostmap       
                                    |                                                                        
                         ___________|______                                                                  
                         |                |                                                                  
                         |                |                                                                  
                    GoalUpdated  ClearEntireCostmap                                                          

The parent node of this subtree is ``PipelineSequence``, which again is a custom Nav2 BT.
While this subtree looks complicated, the crux of the tree can be represented with only one parent and two children nodes like this:

.. code-block::

        PipelineSequence         
                |                
         _______|_________       
         |               |       
         |               |       
 ComputePathToPose  FollowPath   

The other children and leaves of the tree are simply to throttle, handle failures, and handle updated goals.

The ``PipelineSequence`` allows the ``ComputePathToPose`` to be ticked, and once that succeeds, the ``ComputePathToPose`` and ``FollowPath`` to be ticked.
The full description of this control node is in the `configuration guide <../../configuration/packages/bt-plugins/controls/PipelineSequence.html>`_.
In the above distillation of the BT, if ``ComputePathToPose`` or ``FollowPath`` return ``FAILURE``,
the parent ``PipelineSequence`` will also return ``FAILURE`` and will therefore the BT will tick the ``RecoveryFallback`` node.

However, in the full ``NavigateWithReplanning`` subtree, there are a few other nodes to consider.

For example, the ``RateController`` node simply helps keep the BT ticks at the specified frequency. The default frequency for this BT is 1 hz. 
This is done to prevent the BT from hitting the planning server with too many useless requests. Consider changing this frequency to something higher or lower depending on the application and the computational cost of 
calculating the path. 

The next child in this tree is the ``RecoveryNode``, which wraps two children,  the ``ComputePathToPose`` and the ``ReactiveFallback``.
Recall from above that the ``RecoveryNode`` that this will return ``SUCCESS`` 
if ``ComputePathToPose`` returns ``SUCCESS`` or if ``ComputePathToPose`` returns ``FAILURE`` but the ``ReactiveFallback`` returns ``SUCCESS``. 
It will return ``FAILURE`` if both ``ComputePathToPose`` and the ``ReactiveFallback`` returns ``FAILURE``, or if the ``number_of_retries`` is violated (in this case one retry is allowed) .. which will then  cause the BT to enter the ``RecoveryFallback`` subtree.

Consider changing the ``number_of_retries`` parameter in the BT if your application requires more retries before a recovery action is triggered.

The ``ComputePathToPose`` is a simple action client to the ``ComputePathToPose`` ROS 2 action server.
The guide to configure this action node can be found in the `configuration guide <../../configuration/packages/bt-plugins/actions/ComputePathToPose.html>`_.

Finally the ``ReactiveFallback`` node simply will tick it's 2nd child, ``ClearEntireCostmap`` *unless* the state of the condition node ``GoalUpdated`` returns ``SUCCESS`` (when, as the name suggests, the goal is updated).
In essence, the global costmap will be cleared unless the goal has been updated. ``ClearEntireCostmap`` is a recovery action that implements the ``clear_entirely_costmap`` service. 
In this case, the BT has set this to the global costmap, which makes sense as the global costmap would be the costmap that would affect the robot's ability to ``ComputePathToPose``.

For convenience, the ``NavigateWithReplanning`` ASCII representation is below again:

.. code-block::

                                              PipelineSequence                                               
                                                      |                                                      
                             _________________________|____________________________                          
                             |                                                    |                          
                             |                                                    |                          
                      RateController                                        RecoveryNode                     
                             |                                                    |                          
                            _|                                  __________________|_____                     
                            |                                   |                      |                     
                            |                                   |                      |                     
                      RecoveryNode                         FollowPath          ReactiveFallback              
                            |                                                          |                     
         ___________________|________                                       ___________|______               
         |                          |                                       |                |               
         |                          |                                       |                |               
 ComputePathToPose          ReactiveFallback                           GoalUpdated  ClearEntireCostmap       
                                    |                                                                        
                         ___________|______                                                                  
                         |                |                                                                  
                         |                |                                                                  
                    GoalUpdated  ClearEntireCostmap                                                          

Now that we have covered the structure of the first major subtree, the ``ComputePathToPose`` subtree, the ``FollowPath`` subtree is largely symetric.

The ``FollowPath`` action node implements the action client to the ``FollowPath`` ROS 2 action server.
The guide to configure this action node can be found in the `configuration guide <../../configuration/packages/bt-plugins/actions/FollowPath.html>`_.

If the ``FollowPath`` action node returns ``SUCCESS`` then this overall subtree will return ``SUCCESS``,
however if ``FollowPath`` returns ``FAILURE`` then the ``RecoveryNode`` will tick the ``ReactiveFallback``
which will tick ``ClearEntireCostmap`` (local) *unless* the ``GoalUpdated`` return ``SUCCESS``.
The local costmap makes sense to clear in this case as it is the costmap that would impede the robot's ability to follow the path.

In both of these subtrees, checking the ``GoalUpdated`` condition node is what gives this subtree  the name ``NavigateWithReplanning``.

We have now gone completely over the possibilities and actions in the ``NavigateWithReplanning``,
let's move on to the ``RecoveryFallback`` subtree, which will be ticked if the ``NavigateWithReplanning`` overall returns ``FAILURE``. The most likely scenario for 
this subtree to return ``FAILURE`` if the ``number_of_retries`` is violated on the ``RecoveryNode`` that wraps either the ``ComputePathToPose`` action, or the ``FollowPath`` action.

RecoveryFallback
================
The recovery fallback subtree is the second big "half" of the Nav2 default ``navigate_w_replanning_and_recovery.xml`` tree.
In short, this subtree is triggered when the ``NavigateWithReplanning`` subtree returns ``FAILURE`` and this subtree helps select the appropriate recovery to be taken based on how many previous times the recovery and the ``NavigateWithReplanning`` subtree returns ``FAILURE``.

Below is an ASCII representation of the subtree:

.. code-block::

                                        |                                             
                                ReactiveFallback                                      
                                        |                                             
       _________________________________|______                                       
       |                                      |                                       
       |                                      |                                       
  GoalUpdated                            RoundRobin                                   
                                              |                                       
                                   ___________|__________________________             
                                   |                       |     |      |             
                                   |                       |     |      |             
                               Sequence                  Spin  Wait  BackUp           
                                   |                                                  
                        ___________|_________                                         
                        |                   |                                         
                        |                   |                                         
               ClearEntireCostmap  ClearEntireCostmap                                 

The top most parent is ``ReactiveFallback`` which dictates that unless ``GoalUpdated`` returns ``SUCCESS``, tick the 2nd child (in  this case the ``RoundRobin``.
This should look familiar to the replanning portions of the ``NavigateWithReplanning`` tree. This is a common BT pattern to handle the situation "Unless 'this condition' happens,Do action A".

Condition nodes can be very powerful, and other custom NAV2 condition nodes include:
- DistanceTraveled
- GoalReached
- isBatteryLow
- TimeExpired

These condition nodes can be extremely powerful and are typically paired with ``ReactiveFallback``. It can be easy to imagine wrapping this whole ``navigate_w_replanning_and_recovery`` tree
in a ``ReactiveFallback`` with a ``isBatteryLow``condition -- meaning the ``navigate_w_replanning_and_recovery`` tree will execute *unless* the battery becomes low. 

In this case, ``GoalUpdated`` returns ``FAILURE`` (keep in mind this status is checked asynchronously), then the BT moves on to tick the ``RoundRobin`` node.
``RoundRobin`` is a custom Nav2 node. This control node will keep on ticking the subsequent child, until ``SUCCESS`` is achieved.
Before ``RoundRobin`` is explained in detail, let's describe what the ``Sequence`` node is. The ``Sequence`` node will tick both of the ``ClearLocalCostmap`` and if that returns ``SUCCESS`` will return ``ClearGlobalCostmap``.
If either of the children of the ``Sequence`` node returns ``FAILURE`` so will the node itself. Additionally, note that the ``Spin`` and ``BackUp`` nodes are clients to the Nav2 Recovery server.
In case a custom recovery action is needed, it can be useful to refer to the source of ``Spin`` ``BackUp`` and ``Wait`` as a reference.

To explain ``RoundRobin`` more clearly, let us assume that the robot is stuck somewhere and we are in this ``RecoveryFallback`` subtree for the first time:

- In the first time, ``RoundRobin`` will tick it's first child, ``Sequence``. Let's assume that these costmap clearing actions return ``SUCCESS``. 
- Upon the ``SUCCESS`` of the ``Sequence`` child (which just means that the costmaps were correctly cleared), the robot will attempt to renavigate in the ``NavigateWithReplanning`` subtree.
- Let's say that clearing the costmaps were not enough, the robot is **still** stuck. Upon entering the ``RoundRobin`` portion of the ``RecoveryFallback`` subtree, the subtree will tick the next child ``Spin``. ``RoundRobin`` retains a memory of nodes visited, and will **not** try to re-clear the costmaps again in this recovery.
- Regardless if ``Spin`` returns ``FAILURE`` or ``SUCCESS`` the next time this portion of the subtree enters, the next subsequent child will be ticked (in this case ``Wait``), and so on. Upon reaching the last child (in this case ``BackUp``), the node will wrap around and tick the ``ClearCostmapSequence`` again. 

``RoundRobin`` will only overall return ``FAILURE`` if **all** children return ``FAILURE``. 

Further details about the ``RoundRobin`` node can be found in the `configuration guide <../../configuration/packages/bt-plugins/controls/RoundRobin.html>`_.

Custom Action
=============

Adding to Launch File
=====================

Testing
=======

