.. _bt_goal_checker_selector_node:

GoalCheckerSelector
===================

It is used to select the GoalChecker that will be used by the goal_checker server. It subscribes to the ``goal_checker_selector`` topic to receive command messages with the name of the GoalChecker to be used. It is commonly used before of the FollowPathAction. The ``selected_goal_checker`` output port is passed to ``goal_checker_id`` input port of the FollowPathAction. If none is provided on the topic, the ``default_goal_checker`` is used.

Any publisher to this topic needs to be configured with some QoS defined as ``reliable`` and ``transient local``.

.. _bt_navigator: https://github.com/ros-planning/navigation2/tree/main/nav2_bt_navigator

Input Ports
-----------

:topic_name:

  ====== =======
  Type   Default
  ------ -------
  string goal_checker_selector  
  ====== =======

  Description
    	The name of the topic used to received select command messages. This is used to support multiple GoalCheckerSelector nodes. 
      
:default_goal_checker:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	The default value for the selected GoalChecker if no message is received from the input topic.


Output Ports
------------

:selected_goal_checker:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	The output selected GoalChecker id. This selected_goal_checker string is usually passed to the FollowPath behavior via the goal_checker_id input port.


Example
-------

.. code-block:: xml

  <GoalCheckerSelector selected_goal_checker="{selected_goal_checker}" default_goal_checker="precise_goal_checker" topic_name="goal_checker_selector"/>
