.. _bt_path_handler_selector_node:

PathHandlerSelector
===================

It is used to select the PathHandler that will be used by the controller server. It subscribes to the ``path_handler_selector`` topic to receive command messages with the name of the PathHandler to be used. It is commonly used before of the FollowPathAction. The ``selected_path_handler`` output port is passed to ``path_handler_id`` input port of the FollowPathAction. If none is provided on the topic, the ``default_path_handler`` is used.

Any publisher to this topic needs to be configured with some QoS defined as ``reliable`` and ``transient local``.

.. _bt_navigator: https://github.com/ros-navigation/navigation2/tree/main/nav2_bt_navigator

Input Ports
-----------

:topic_name:

  ====== =====================
  Type   Default
  ------ ---------------------
  string path_handler_selector
  ====== =====================

  Description
      The name of the topic used to received select command messages. This is used to support multiple PathHandlerSelector nodes.

:default_path_handler:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
      The default value for the selected PathHandler if no message is received from the input topic.


Output Ports
------------

:selected_path_handler:

  ====== =======
  Type   Default
  ------ -------
  string N/A
  ====== =======

  Description
      The output selected PathHandler id. This selected_path_handler string is usually passed to the FollowPath behavior via the path_handler_id input port.


Example
-------
.. tabs::

  .. group-tab:: Lyrical and newer

    .. code-block:: xml

      <PathHandlerSelector selected_path_handler="{selected_path_handler}" default_path_handler="path_handler" topic_name="path_handler_selector"/>

  .. group-tab:: Kilted and older

    .. code-block:: xml

      <PathHandlerSelector selected_path_handler="{selected_path_handler}" default_path_handler="PathHandler" topic_name="path_handler_selector"/>
