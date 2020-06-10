.. bt_decorators:

RateController
==============

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
    