.. _bt_recovery_node_control:

RecoveryNode
============

The RecoveryNode is a control flow node with two children. 
It returns SUCCESS if and only if the first child returns SUCCESS. 
The second child will be executed only if the first child returns FAILURE. 
If the second child SUCCEEDS, then the first child will be executed again. 
The user can specify how many times the recovery actions should be taken before returning FAILURE. 
In nav2, the RecoveryNode is included in Behavior Trees to implement recovery actions upon failures.

Input Ports
-----------

:number_of_retries:

  ==== =======
  Type Default
  ---- -------
  int  1
  ==== =======

  Description
    	Number of retries.

Example
-------

.. code-block:: xml

    <RecoveryNode number_of_retries="1">
        <!--Add tree components here--->
    </RecoveryNode>
