.. _build-instructions:

Build and Install
#################

Install
*******

Nav2 and its dependencies are released as binaries.
You may install it via the following to get the latest stable released version:

  ``sudo apt install ros-<distro>-navigation2 ros-<distro>-nav2-bringup ros-<distro>-turtlebot3*``

(For Ubuntu 20.04 use this command as the parsing of wildcards have been changed:

  ``sudo apt install ros-<distro>-navigation2 ros-<distro>-nav2-bringup '~ros-<distro>-turtlebot3-.*'``


Build
*****

There are 3 ways to build Nav2.
Building for a specific released distribution (e.g. ``foxy``, ``galactic``), build Nav2 on main branch using a quickstart setup script, or building main branch manually.

.. rst-class:: content-collapse

For Released Distributions
==========================

Install ROS
-----------

Please install ROS 2 via the usual `install instructions <https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Binary.html>`_ for your desired distribution.
Note: ``<ros2-distro>-devel`` was the branch naming schema pre-``galactic``.
For Galactic and newer, it is simply ``<ros2-distro>``.

Build Nav2
----------

We're going to create a new workspace, ``nav2_ws``, clone our Nav2 branch into it, and build.
``rosdep`` will be used to get the dependency binaries for Nav2 in your specific distribution.

.. code:: bash

  mkdir -p ~/nav2_ws/src
  cd ~/nav2_ws/src
  git clone https://github.com/ros-planning/navigation2.git --branch <ros2-distro>-devel
  cd ~/nav2_ws
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>
  colcon build --symlink-install

Note: You need to change ``--rosdistro`` to the selected ROS 2 distribution name (e.g ``foxy``, ``galactic``).



.. rst-class:: content-collapse


For Main Branch Development
===========================

Build ROS 2 Main
----------------
Build or install ROS 2 ``rolling`` using the `build instructions <https://docs.ros.org/en/rolling/Installation/Ubuntu-Development-Setup.html>`_ provided in the ROS 2 documentation.
All development is done using the ``rolling`` distribution on Nav2's ``main`` branch and cherry-picked over to released distributions during syncs (if ABI compatible).

Build Nav2 Main
---------------

Now that ROS 2 ``rolling`` is installed, we have to install our dependencies and build Nav2 itself. 
We'll create a new workspace, ``nav2_ws`` and clone the Nav2 project into it.
Afterwards, we'll use ``rosdep`` to automatically find and install our dependencies that were not included in the core ROS 2 install itself (``behaviortree.CPP``, ``ompl``, etc).
If you would like to use a custom version of any of these dependencies, simply overlay them in your ``nav2_ws`` and it will use those rather than the binary installed versions.

.. code:: bash

  mkdir -p ~/nav2_ws/src
  cd ~/nav2_ws/src
  git clone https://github.com/ros-planning/navigation2.git --branch main
  cd ~/nav2_ws
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro rolling
  colcon build --symlink-install
  source install/setup.bash

You are now ready for the demonstrations!

Docker
******

The official Dockerhub entries are primarily for use in the Nav2 CI, but they may also be used for development. It is useful to have a docker image that tracks Nav2 ``main`` branch. The ``Dockerfile`` in the root of the repository is recommended for production use, set to your distribution of choice.

It is though generally recomended to install Nav2 releases from the apt repository inside a container if you'd like to use our released binaries.

.. rst-class:: content-collapse

Building Docker Container
=========================

To build an image from the Dockerfile in the Nav2 folder:
First, clone the repo to your local system (or see Building the source above)


.. code:: bash

  sudo docker build -t nav2/latest .

If proxies are needed:

.. code:: bash

  sudo docker build -t nav2/latest --build-arg http_proxy=http://proxy.my.com:### --build-arg https_proxy=http://proxy.my.com:### .

Note: You may also need to configure your docker for DNS to work. See article here for details: https://development.robinwinslow.uk/2016/06/23/fix-docker-networking-dns/

If you would like to build from dockerhub cache to speed up the build

.. code:: bash

  sudo docker pull rosplanning/navigation2:main
  sudo docker build -t nav2/latest --cache-from rosplanning/navigation2:main .

.. rst-class:: content-collapse

Using DockerHub Container
=========================
We allow for you to pull the latest docker image from the main branch at any time. As new releases and tags are made, docker containers on docker hub will be versioned as well to chose from.
This docker image will not contain a built overlay, and you must build the overlay Nav2 workspace yourself (see Build Nav2 Main up above).

.. code:: bash

  sudo docker pull rosplanning/navigation2:main

!!!!

Generate Doxygen
****************

Run ``doxygen`` in the root of the Nav2 repository.
It will generate a ``/doc/*`` directory containing the documentation.
The documentation entrypoint in a browser is index.html.

!!!!

Help
****

:ref:`build-troubleshooting-guide`


.. toctree::
   :hidden:

   build_docs/build_troubleshooting_guide.rst
