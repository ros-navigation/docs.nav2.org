.. _get_backtrace:

Get Backtrace in ROS 2 / Nav2
*****************************

- `Overview`_
- `Preliminaries`_
- `From a Node`_
- `From a Launch File`_
- `From Nav2 Bringup`_
- `Automatic backtrace on crash`_

Overview
========

This document explains one set of methods for getting backtraces for ROS 2 and Nav2.
There are many ways to accomplish this, but this is a good starting point for new C++ developers without GDB experience.

The following steps show ROS 2 users how to modify the Nav2 stack to get traces from specific servers when they encounter a problem.
This tutorial applies to both simulated and physical robots.

This will cover how to get a backtrace from a specific node using ``ros2 run``, from a launch file representing a single node using ``ros2 launch``, and from a more complex orchestration of nodes.
By the end of this tutorial, you should be able to get a backtrace when you notice a server crashing in ROS 2.

Preliminaries
=============

GDB is the most popular debugger for C++ on Unix systems.
It can be used to determine the reason for a crash and track threads.
It may also be used to add breakpoints in your code to check values in memory a particular points in your software.

Using GDB is a critical skill for all software developers working on C/C++.
Many IDEs will have some kind of debugger or profiler built in, but with ROS, there are few IDEs to choose.
Therefore it's important to understand how to use these raw tools you have available rather than relying on an IDE to provide them.
Further, understanding these tools is a fundamental skill of C/C++ development and leaving it up to your IDE can be problematic if you change roles and no longer have access to it or are doing development on the fly through an ssh session to a remote asset.

Using GDB luckily is fairly simple after you have the basics under your belt.
The first step is to add ``-g`` to your compiler flags for the ROS package you want to profile / debug.
This flag builds debug symbols that GDB and valgrind can read to tell you specific lines of code in your project are failing and why.
If you do not set this flag, you can still get backtraces but it will not provide line numbers for failures.
Be sure to remove this flag after debugging, it will slow down performance at run-time.

Adding the following line to your ``CMakeLists.txt`` for your project should do the trick.
If your project already has a ``add_compile_options()``, you can simply add ``-g`` to it.
Then simply rebuild your workspace with this package ``colcon build --packages-select <package-name>``.
It may take a little longer than usual to compile.

.. code-block:: cmake

  add_compile_options(-g)

Now you're ready to debug your code!
If this was a non-ROS project, at this point you might do something like below.
Here we're launching a GDB session and telling our program to immediately run.
Once your program crashes, it will return a gdb session prompt denoted by ``(gdb)``.
At this prompt you can access the information you're interested in.
However, since this is a ROS project with lots of node configurations and other things going on, this isn't a great option for beginners or those that don't like tons of commandline work and understanding the filesystem.

.. code-block:: bash

  gdb ex run --args /path/to/exe/program

Below are sections to describe the 3 major situations you could run into with ROS 2-based systems.
Read the section that best describes the problem you're attempting to solve.

From a Node
===========

Just as in our non-ROS example, we need to setup a GDB session before launching our ROS 2 node.
While we could set this up through the commandline with some knowledge of the ROS 2 file system, we can instead use the launch ``--prefix`` option the kind folks at Open Robotics provided for us.

``--prefix`` will execute some bits of code before our ``ros2`` command allowing us to insert some information.
If you attempted to do ``gdb ex run --args ros2 run <pkg> <node>`` as analog to our example in the preliminaries, you'd find that it couldn't find the ``ros2`` command.
If you're even more clever, you'd find that trying to source your workspace would also fail for similar reasons.

Rather than having to revert to finding the install path of the executable and typing it all out, we can instead use ``--prefix``.
This allows us to use the same ``ros2 run`` syntax you're used to without having to worry about some of the GDB details.

.. code-block:: bash

  ros2 run --prefix 'gdb -ex run --args' <pkg> <node> --all-other-launch arguments

Just as before, this prefix will launch a GDB session and run the node you requested with all the additional commandline arguments.
You should now have your node running and should be chugging along with some debug printing.

Once your server crashes, you'll see a prompt like below. At this point you can now get a backtrace.

.. code-block:: bash

  (gdb)

In this session, type ``backtrace`` and it will provide you with a backtrace.
Copy this for your needs.
For example:

.. code-block:: bash

  (gdb) backtrace
  #0  __GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:50
  #1  0x00007ffff79cc859 in __GI_abort () at abort.c:79
  #2  0x00007ffff7c52951 in ?? () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
  #3  0x00007ffff7c5e47c in ?? () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
  #4  0x00007ffff7c5e4e7 in std::terminate() () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
  #5  0x00007ffff7c5e799 in __cxa_throw () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
  #6  0x00007ffff7c553eb in ?? () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
  #7  0x000055555555936c in std::vector<int, std::allocator<int> >::_M_range_check (
      this=0x5555555cfdb0, __n=100) at /usr/include/c++/9/bits/stl_vector.h:1070
  #8  0x0000555555558e1d in std::vector<int, std::allocator<int> >::at (this=0x5555555cfdb0,
      __n=100) at /usr/include/c++/9/bits/stl_vector.h:1091
  #9  0x000055555555828b in GDBTester::VectorCrash (this=0x5555555cfb40)
      at /home/steve/Documents/nav2_ws/src/gdb_test_pkg/src/gdb_test_node.cpp:44
  #10 0x0000555555559cfc in main (argc=1, argv=0x7fffffffc108)
      at /home/steve/Documents/nav2_ws/src/gdb_test_pkg/src/main.cpp:25

In this example you should read this in the following way, starting at the bottom:

- In the main function, on line 25 we call a function VectorCrash.

- In VectorCrash, on line 44, we crashed in the Vector's ``at()`` method with input ``100``.

- It crashed in ``at()`` on STL vector line 1091 after throwing an exception from a range check failure.

These traces take some time to get used to reading, but in general, start at the bottom and follow it up the stack until you see the line it crashed on.
Then you can deduce why it crashed.
When you are done with GDB, type ``quit`` and it will exit the session and kill any processes still up.
It may ask you if you want to kill some threads at the end, say yes.

From a Launch File
==================

Just as in our non-ROS example, we need to setup a GDB session before launching our ROS 2 launch file.
While we could set this up through the commandline, we can instead make use of the same mechanics that we did in the ``ros2 run`` node example, now using a launch file.

In your launch file, find the node that you're interested in debugging.
For this section, we assume that your launch file contains only a single node (and potentially other information as well).
The ``Node`` function used in the ``launch_ros`` package will take in a field ``prefix`` taking a list of prefix arguments.
We will insert the GDB snippet here with one change from our node example, use of ``xterm``.
``xterm`` will pop up a new terminal window to show and interact with GDB.
We do this because of issues handling ``stdin`` on launch files (e.g. if you hit control+C, are you talking to GDB or launch?).
See `this ticket <https://github.com/ros2/launch_ros/issues/165>`_ for more information.
See below for an example debugging SLAM Toolbox.

.. code-block:: python

  start_sync_slam_toolbox_node = Node(
      parameters=[
        get_package_share_directory("slam_toolbox") + '/config/mapper_params_online_sync.yaml',
        {'use_sim_time': use_sim_time}
      ],
      package='slam_toolbox',
      executable='sync_slam_toolbox_node',
      name='slam_toolbox',
      prefix=['xterm -e gdb -ex run --args'],
      output='screen')

Just as before, this prefix will launch a GDB session, now in ``xterm`` and run the launch file you requested with all the additional launch arguments defined.

Once your server crashes, you'll see a prompt like below, now in the ``xterm`` session. At this point you can now get a backtrace.

.. code-block:: bash

  (gdb)

In this session, type ``backtrace`` and it will provide you with a backtrace.
Copy this for your needs.
See the example trace in the section above for an example.

These traces take some time to get used to reading, but in general, start at the bottom and follow it up the stack until you see the line it crashed on.
Then you can deduce why it crashed.
When you are done with GDB, type ``quit`` and it will exit the session and kill any processes still up.
It may ask you if you want to kill some threads at the end, say yes.

From Large Project
==================

Working with launch files with multiple nodes is a little different so you can interact with your GDB session without being bogged down by other logging in the same terminal.
For this reason, when working with larger launch files, its good to pull out the specific server you're interested in and launching it separately.
These instructions are targeting Nav2, but are applicable to any large project with many nodes of any type in a series of launch file(s).

As such, for this case, when you see a crash you'd like to investigate, its beneficial to separate this server from the others.

If your server of interest is being launched from a nested launch file (e.g. an included launch file) you may want to do the following:

- Comment out the launch file inclusion from the parent launch file

- Recompile the package of interest with ``-g`` flag for debug symbols

- Launch the parent launch file in a terminal

- Launch the server's launch file in another terminal following the instructions in `From a Launch File`_.

Alternatively, if you server of interest is being launched in these files directly (e.g. you see a ``Node``, ``LifecycleNode``, or inside a ``ComponentContainer``), you will need to separate this from the others:

- Comment out the node's inclusion from the parent launch file

- Recompile the package of interest with ``-g`` flag for debug symbols

- Launch the parent launch file in a terminal

- Launch the server's node in another terminal following the instructions in `From a Node`_.

.. note::
  Note that in this case, you may need to remap or provide parameter files to this node if it was previously provided by the launch file. Using ``--ros-args`` you can give it the path to the new parameters file, remaps, or names. See `this ROS 2 tutorial <https://docs.ros.org/en/rolling/Guides/Node-arguments.html>`_ for the commandline arguments required.

  We understand this can be a pain, so it might encourage you to rather have each node possible as a separately included launch file to make debugging easier. An example set of arguments might be ``--ros-args -r __node:=<node_name> --params-file /absolute/path/to/params.yaml`` (as a template).

Once your server crashes, you'll see a prompt like below in the specific server's terminal. At this point you can now get a backtrace.

.. code-block:: bash

  (gdb)

In this session, type ``backtrace`` and it will provide you with a backtrace.
Copy this for your needs.
See the example trace in the section above for an example.

These traces take some time to get used to reading, but in general, start at the bottom and follow it up the stack until you see the line it crashed on.
Then you can deduce why it crashed.
When you are done with GDB, type ``quit`` and it will exit the session and kill any processes still up.
It may ask you if you want to kill some threads at the end, say yes.

From Nav2 Bringup
=================

To debug directly from the nav2 bringup launch files you may want to do the following:

- Add ``prefix=['xterm -e gdb -ex run --args']`` to the non-composed node in the appropriate launch file.

- Recompile the package of interest with ``-g`` flag for debug symbols.

- Launch normally with ``ros2 launch nav2_bringup tb3_simulation_launch.py use_composition:=False``. A separate xterm window will open with the process of interest running in gdb.

.. note::
  Turning off composition has serious performance impacts. If this is important to you please follow "From Large Project".

Once your server crashes, you'll see a prompt like below in the xterm window. At this point you can now get a backtrace.

.. code-block:: bash

  (gdb)

In this session, type ``backtrace`` and it will provide you with a backtrace.
Copy this for your needs.
See the example trace in the section above for an example.

These traces take some time to get used to reading, but in general, start at the bottom and follow it up the stack until you see the line it crashed on.
Then you can deduce why it crashed.
When you are done with GDB, type ``quit`` and it will exit the session and kill any processes still up.
It may ask you if you want to kill some threads at the end, say yes.

Automatic backtrace on crash
============================

The `backward-cpp <https://github.com/bombela/backward-cpp>`_ library provides beautiful stack traces, and the `backward_ros <https://github.com/pal-robotics/backward_ros/tree/foxy-devel>`_ wrapper simplifies its integration.

Navigation2 packages come pre-compiled with backward_ros, so you can automatically get a backtrace when a process crashes. This changes the logs to include a backtrace when the process crashes. Instead of getting a simple error log such as:

.. code-block:: bash

  [planner_server-13] [INFO] [1754377648.634265021] [planner_server]: Computing path to goal....
  [ERROR] [planner_server-13]: process has died [pid 165734, exit code -11, cmd '/opt/overlay_ws/src/navigation2/install/nav2_planner/lib/nav2_planner/planner_server --ros-args --log-level info --ros-args -r __node:=planner_server -r __ns:=/ -p use_sim_time:=True --params-file /tmp/launch_params_ou47e26i -r /tf:=tf -r /tf_static:=tf_static'].

You get a more detailed output with a backtrace, which can help you identify the source of the crash:

.. code-block:: bash

  [planner_server-13] [INFO] [1754463292.960594359] [planner_server]: Computing path to goal....
  [planner_server-13] Stack trace (most recent call last) in thread 130805:
  [planner_server-13] #11   Object "/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2", at 0xffffffffffffffff, in
  [planner_server-13] #10   Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7509de004c3b, in
  [planner_server-13] #9    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7509ddf77aa3, in
  [planner_server-13] #8    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33", at 0x7509de207db3, in
  [planner_server-13] #7    Object "/opt/overlay_ws/src/navigation2/install/nav2_planner/lib/libplanner_server_core.so", at 0x7509de887ee7, in std::__future_base::_Async_state_impl<std::thread::_Invoker<std::tuple<nav2::SimpleActionServer<nav2_msgs::action::ComputePathToPose>::handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputePathToPose> >)::{lambda()#1}> >, void>::_M_run()
  [planner_server-13] #6    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x7509ddf7ced2, in
  [planner_server-13] #5    Object "/opt/overlay_ws/src/navigation2/install/nav2_planner/lib/libplanner_server_core.so", at 0x7509de8825cc, in std::__future_base::_State_baseV2::_M_do_set(std::function<std::unique_ptr<std::__future_base::_Result_base, std::__future_base::_Result_base::_Deleter> ()>*, bool*)
  [planner_server-13] #4    Object "/opt/overlay_ws/src/navigation2/install/nav2_planner/lib/libplanner_server_core.so", at 0x7509de8bd380, in std::_Function_handler<std::unique_ptr<std::__future_base::_Result_base, std::__future_base::_Result_base::_Deleter> (), std::__future_base::_Task_setter<std::unique_ptr<std::__future_base::_Result<void>, std::__future_base::_Result_base::_Deleter>, std::thread::_Invoker<std::tuple<nav2::SimpleActionServer<nav2_msgs::action::ComputePathToPose>::handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputePathToPose> >)::{lambda()#1}> >, void> >::_M_invoke(std::_Any_data const&)
  [planner_server-13] #3    Object "/opt/overlay_ws/src/navigation2/install/nav2_planner/lib/libplanner_server_core.so", at 0x7509de8bc2f4, in nav2::SimpleActionServer<nav2_msgs::action::ComputePathToPose>::work()
  [planner_server-13] #2    Object "/opt/overlay_ws/src/navigation2/install/nav2_planner/lib/libplanner_server_core.so", at 0x7509de87b798, in nav2_planner::PlannerServer::computePlan()
  [planner_server-13] #1    Object "/opt/overlay_ws/src/navigation2/install/nav2_planner/lib/libplanner_server_core.so", at 0x7509de873ce4, in nav2_planner::PlannerServer::publishPlan(nav_msgs::msg::Path_<std::allocator<void> > const&)
  [planner_server-13] #0    Object "/opt/ros/rolling/lib/librclcpp_lifecycle.so", at 0x7509de62bf94, in rclcpp_lifecycle::SimpleManagedEntity::is_activated() const
  [planner_server-13] Segmentation fault (Address not mapped to object [0x8])
  [ERROR] [planner_server-13]: process has died [pid 129889, exit code -11, cmd '/opt/overlay_ws/src/navigation2/install/nav2_planner/lib/nav2_planner/planner_server --ros-args --log-level info --ros-args -r __node:=planner_server -r __ns:=/ -p use_sim_time:=True --params-file /tmp/launch_params_apyru5sq -r /tf:=tf -r /tf_static:=tf_static'].

Building Nav2 with the **RelWithDebInfo** or **Debug** build configuration allows for even more detailed backtraces including line numbers, file names, and relevant code blocks, in the backtrace:

.. code-block:: bash

  [planner_server-13] [INFO] [1754377736.955537475] [planner_server]: Computing path to goal....
  [planner_server-13] Stack trace (most recent call last) in thread 173639:
  [planner_server-13] #11   Object "/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2", at 0xffffffffffffffff, in
  [planner_server-13] #10   Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x795ea7ac8c3b, in
  [planner_server-13] #9    Object "/usr/lib/x86_64-linux-gnu/libc.so.6", at 0x795ea7a3baa3, in

  ⋮

  [planner_server-13] #2    Source "/opt/overlay_ws/src/navigation2/nav2_planner/src/planner_server.cpp", line 546, in computePlan [0x795ea8344363]
  [planner_server-13]         543:     }
  [planner_server-13]         544:
  [planner_server-13]         545:     // Publish the plan for visualization purposes
  [planner_server-13]       > 546:     publishPlan(result->path);
  [planner_server-13]         547:
  [planner_server-13]         548:     auto cycle_duration = this->now() - start_time;
  [planner_server-13]         549:     result->planning_time = cycle_duration;
  [planner_server-13] #1    Source "/opt/overlay_ws/src/navigation2/nav2_planner/src/planner_server.cpp", line 640, in publishPlan [0x795ea833df4a]
  [planner_server-13]         637:   auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  [planner_server-13]         638:   RCLCPP_WARN(get_logger(), "Publishing plan with %zu poses", path.poses.size());
  [planner_server-13]         639:   plan_publisher_.reset();
  [planner_server-13]       > 640:   if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
  [planner_server-13]         641:     plan_publisher_->publish(std::move(msg));
  [planner_server-13]         642:   }
  [planner_server-13]         643: }
  [planner_server-13] #0    Object "/opt/ros/rolling/lib/librclcpp_lifecycle.so", at 0x795ea80f1f94, in rclcpp_lifecycle::SimpleManagedEntity::is_activated() const
  [planner_server-13] Segmentation fault (Address not mapped to object [0x8])
  [ERROR] [planner_server-13]: process has died [pid 172784, exit code -11, cmd '/opt/overlay_ws/src/navigation2/install/nav2_planner/lib/nav2_planner/planner_server --ros-args --log-level info --ros-args -r __node:=planner_server -r __ns:=/ -p use_sim_time:=True --params-file /tmp/launch_params_px13xl6j -r /tf:=tf -r /tf_static:=tf_static'].

If you want to enable automatic backtraces in your own packages, just add ``backward_ros`` as a dependency and ``find_package`` in your CMakeLists, then the backward libraries will be injected in all your executables and libraries.

**Note:** Currently, backward_ros does not work with ComposedNodes due to missing support in rclcpp. Until this `issue <https://github.com/ros2/rclcpp/issues/2396>`_ is resolved, launching the Nav2 stack with composition disabled (e.g. with a `use_composition:=False` launch argument) is required to get automatic backtraces.
