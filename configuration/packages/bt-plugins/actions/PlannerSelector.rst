.. _bt_planner_selector_node:

PlannerSelector
===============

It is used to select the planner that will be used by the planner server. It subscribes to the ``planner_selector`` topic to receive command messages with the name of the planner to be used. It is commonly used before of the ComputePathToPoseAction. The ``selected_planner`` output port is passed to ``planner_id`` input port of the ComputePathToPoseAction. If none is provided on the topic, the ``default_planner`` is used.

Any publisher to this topic needs to be configured with some QoS defined as ``reliable`` and ``transient local``.

.. _bt_navigator: https://github.com/ros-planning/navigation2/tree/main/nav2_bt_navigator

Input Ports
-----------

:topic_name:

  ====== =======
  Type   Default
  ------ -------
  string planner_selector  
  ====== =======

  Description
    	The name of the topic used to received select command messages. This is used to support multiple PlannerSelector nodes.
      
:default_planner:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	The default value for the selected planner if no message is received from the input topic.


Output Ports
------------

:selected_planner:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	The output selected planner id. This selected_planner string is usually passed to the ComputePathToPose behavior via the planner_id input port.


Example
-------

.. code-block:: xml

  <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector"/>
