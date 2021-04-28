.. _bt_controller_selector_node:

ControllerSelector
==================

It is used to select the Controller that will be used by the Controller server. It subscribes to the ``controller_selector`` topic to receive command messages with the name of the Controller to be used. It is commonly used before of the FollowPathAction. The ``selected_controller`` output port is passed to ``controller_id`` input port of the FollowPathAction. If none is provided on the topic, the ``default_controller`` is used.

Any publisher to this topic needs to be configured with some QoS defined as ``reliable`` and ``transient local``.

.. _bt_navigator: https://github.com/ros-planning/navigation2/tree/main/nav2_bt_navigator

Input Ports
-----------

:topic_name:

  ====== =======
  Type   Default
  ------ -------
  string controller_selector  
  ====== =======

  Description
    	The name of the topic used to received select command messages. This is used to support multiple ControllerSelector nodes. 
      
:default_controller:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	The default value for the selected Controller if no message is received from the input topic.


Output Ports
------------

:selected_controller:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	The output selected Controller id. This selected_controller string is usually passed to the FollowPath behavior via the controller_id input port.


Example
-------

.. code-block:: xml

  <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
