.. _bt_is_path_valid_condition:

IsPathValid
===========

Checks to see if the global path is valid. If there is a
obstacle along the path, the condition returns FAILURE, otherwise
it returns SUCCESS. 

Example
-------

.. code-block:: xml

    <IsPathValid server_timeout="10" path="{path}"/>