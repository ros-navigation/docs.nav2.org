.. _bt_smoother_selector_node:

SmootherSelector
==================

It is used to select the Smoother that will be used by the Smoother server. It subscribes to the ``smoother_selector`` topic to receive command messages with the name of the Smoother to be used. It is commonly used before of the FollowPathAction. If none is provided on the topic, the ``default_smoother`` is used.

Any publisher to this topic needs to be configured with some QoS defined as ``reliable`` and ``transient local``.

.. _bt_navigator: https://github.com/ros-planning/navigation2/tree/main/nav2_bt_navigator

Input Ports
-----------

:topic_name:

  ====== =======
  Type   Default
  ------ -------
  string smoother_selector  
  ====== =======

  Description
    	The name of the topic used to received select command messages. This is used to support multiple SmootherSelector nodes. 
      
:default_smoother:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	The default value for the selected Smoother if no message is received from the input topic.


Output Ports
------------

:selected_smoother:

  ====== =======
  Type   Default
  ------ -------
  string N/A  
  ====== =======

  Description
    	The output selected Smoother id.


Example
-------

.. code-block:: xml

  <SmootherSelector selected_smoother="{selected_smoother}" default_smoother="SimpleSmoother" topic_name="smoother_selector"/>
