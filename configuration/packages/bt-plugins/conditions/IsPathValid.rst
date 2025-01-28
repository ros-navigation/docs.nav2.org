.. _bt_is_path_valid_condition:

IsPathValid
===========

Checks to see if the global path is valid. If there is a
obstacle along the path, the condition returns FAILURE, otherwise
it returns SUCCESS. Additionally, users can specify a threshold cost(0-254)
for path validity. If the path cost exceeds this threshold, the condition will also return FAILURE.

Example
-------

.. code-block:: xml

    <IsPathValid server_timeout="10" path="{path}" max_cost="100"/>