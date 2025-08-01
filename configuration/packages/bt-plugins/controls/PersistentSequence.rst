.. _bt_persistent_sequence_control:

PersistentSequence
===================

The PersistentSequenceNode is similar to the SequenceNode, but it stores the index of the last running child in the blackboard (key: `current_child_idx`), and it does not reset the index when it got halted. It used to tick children in an ordered sequence. If any child returns RUNNING, previous children will NOT be ticked again.
This can be helpful paired with the ``PauseResumeController``.

- If all the children return SUCCESS, this node returns SUCCESS.
- If a child returns RUNNING, this node returns RUNNING. Loop is NOT restarted, the same running child will be ticked again.
- If a child returns FAILURE, stop the loop and return FAILURE. Restart the loop only if (reset_on_failure == true)

Example
-------

.. code-block:: xml

    <Script code="current_child_idx := 0" />
    <PersistentSequence current_child_idx="{current_child_idx}">
        <!-- Child nodes here -->
    </PersistentSequence>
