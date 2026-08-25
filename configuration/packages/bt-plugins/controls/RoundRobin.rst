.. _bt_round_robin_control:


RoundRobin
==========

Custom control flow node used to create a round-robin behavior for children BT nodes.

Input Ports
-----------

:wrap_around:

  ============= =======
  Type          Default
  ============= =======
  bool          false
  ============= =======

  Description
    Controls wrap-around behavior. When ``false``, the node returns FAILURE instead of wrapping to the first child after all children have been attempted. When ``true``, the node wraps around to the first child and continues the round-robin behavior.

Example
-------

.. code-block:: xml

    <RoundRobin wrap_around="false">
        <!--Add tree components here--->
    </RoundRobin>
