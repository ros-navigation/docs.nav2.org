.. _bt_concatenate_paths_action:

ConcatenatePaths
================

Concatenates two paths into a single path, in order such that the output is ``input_path1 + input_path2``.
May be used with multiple of these calls sequentially to concatenate multiple paths.

Input Ports
-----------

:input_path1:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  nav_msgs/Path                   N/A
  =============================== =======

  Description
        First path to concatenate.

:input_path2:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  nav_msgs/Path                   N/A
  =============================== =======

  Description
        Second path to concatenate.

Output Ports
------------

:input_path2:

  =============================== =======
  Type                            Default
  ------------------------------- -------
  nav_msgs/Path                   N/A
  =============================== =======

  Description
        Output concatenated path.

Example
-------

.. code-block:: xml

    <ConcatenatePaths input_path1="{main_path}" input_path2="{last_mile_path}" output_path="{path}"/>
