.. _bt_progress_checker_selector_node:

ProgressCheckerSelector
=======================

It is used to select the ProgressChecker that will be used by the progress_checker server. It subscribes to the ``progress_checker_selector`` topic to receive command messages with the name of the ProgressChecker to be used. It is commonly used before of the FollowPathAction. The ``selected_progess_checker`` output port is passed to ``progress_checker_id`` input port of the FollowPathAction. If none is provided on the topic, the ``default_progress_checker`` is used.

Any publisher to this topic needs to be configured with some QoS defined as ``reliable`` and ``transient local``.

.. _bt_navigator: https://github.com/ros-navigation/navigation2/tree/main/nav2_bt_navigator

Input Ports
-----------

:topic_name:

  ====== =======
  Type   Default
  ------ -------
  string progress_checker_selector  
  ====== =======

  Description
    	The name of the topic used to received select command messages. This is used to support multiple ProgressCheckerSelector nodes. 
      
:default_progress_checker:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	The default value for the selected ProgressChecker if no message is received from the input topic.


Output Ports
------------

:selected_progress_checker:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	The output selected ProgressChecker id. This selected_progress_checker string is usually passed to the FollowPath behavior via the progress_checker_id input port.


Example
-------

.. code-block:: xml

  <ProgressCheckerSelector selected_progress_checker="{selected_progress_checker}" default_progress_checker="precise_progress_checker" topic_name="progress_checker_selector"/>
