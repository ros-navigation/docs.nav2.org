.. _bt_is_path_valid_condition:

IsPathValid
=======

Checks to see if the global path is valid. If there is a
LETHAL obstacle along the path, the condition returns FAILURE, otherwise
it returns SUCCESS. 

Example
-------

.. code-block:: xml

    <IsPathValid path="{path}"/>