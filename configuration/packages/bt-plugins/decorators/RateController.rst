.. bt_decorators:

RateController
==============

A node that throttles the tick rate for its child. 
The tick rate can be supplied to the node as a parameter. 
The node returns RUNNING when it is not ticking its child. 
Currently, in the navigation stack, the ``RateController`` is 
used to adjust the rate at which the ``ComputePathToPose`` and ``GoalReached`` nodes are ticked.

Input Ports
-----------

:hz:

  ====== =======
  Type   Default
  ------ -------
  double  10.0
  ====== =======

  Description
        Rate to throttle an action or a group of actions.

Example
-------

.. code-block:: xml

    <RateController hz="1.0">
        <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
        </RecoveryNode>
    </RateController>
    