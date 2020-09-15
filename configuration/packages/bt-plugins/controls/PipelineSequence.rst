.. _bt_pipe_line_sequence_control:

PipelineSequence
================

Ticks the first child till it succeeds, then ticks the first and second children till the second one succeeds. 
It then ticks the first, second, and third children until the third succeeds, and so on, and so on. If at any 
time a child returns RUNNING, that doesn't change the behavior. If at any time a child returns FAILURE, that 
stops all children and returns FAILURE overall.


Example
-------

.. code-block:: xml

    <PipelineSequence>
        <!--Add tree components here--->
    </PipelineSequence>
    