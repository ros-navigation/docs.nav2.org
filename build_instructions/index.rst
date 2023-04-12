.. _build-instructions:

Build and Install
#################

Install
*******

Nav2 and its dependencies are released as binaries.
You may install it via the following to get the latest stable released version:

.. code:: bash

  source /opt/ros/<distro>/setup.bash
  sudo apt install \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-turtlebot3*

Build
*****

There are a few ways to build Nav2:

* Using Released Distribution Binaries
* Using Rolling Development Source

.. tip::
  For a *repeatable*, *reproducible* and *streamlined* development experience, check the Nav2 documentation on using :ref:`dev-containers`!

.. rst-class:: content-collapse

Released Distribution Binaries
==============================

To build Nav2, you'll first need to build or install ROS 2 and related development tools, including: ``colcon``, ``rosdep`` and ``vcstool``.

.. seealso::
  For more information on building or installing ROS 2 distros, see the official documentation:

  * `ROS 2 Installation <https://docs.ros.org/en/rolling/Installation.html>`_
  * `Install development tools and ROS tools <https://docs.ros.org/en/rolling/Installation/Alternatives/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools>`_

Once your environment is setup, clone the repo, install all dependencies, and build the workspace:

.. attention::
   For ROS 2 distros prior to ``galactic``, the branch naming schema was ``<distro>-devel``.

.. code:: bash

  source /opt/ros/<distro>/setup.bash
  export NAV2_WS=~/nav2_ws
  mkdir -p $NAV2_WS/src && cd $NAV2_WS
  git clone https://github.com/ros-planning/navigation2.git --branch $ROS_DISTRO ./src/navigation2
  rosdep install -y \
    --from-paths ./src \
    --ignore-src
  colcon build \
    --symlink-install

You can then ``source $NAV2_WS/install/setup.bash`` to get ready for demonstrations!

.. hint::
  For more examples on building Nav2 from released distribution binaries, checkout `distro.Dockerfile <https://github.com/ros-planning/navigation2/blob/main/tools/distro.Dockerfile>`_.

.. rst-class:: content-collapse

Using Rolling Development Source
================================

Building Nav2 using rolling development source is similar to building Nav2 from released distribution binaries, where instead you build dependencies from source using the main development branches for all ROS based packages.

.. seealso::
  For more information on building ROS 2 from source, see the official documentation:

  * `ROS 2 Building from source <https://docs.ros.org/en/rolling/Installation.html#building-from-source>`_

Once your environment is setup, clone the repo, import all dependencies, and build the workspace:

.. attention::
   Be sure to check that all dependencies you need are included and uncommented in the ``.repos`` file.

.. code:: bash

  source <ros_ws>/install/setup.bash
  export NAV2_WS=~/nav2_ws
  mkdir -p $NAV2_WS/src && cd $NAV2_WS
  git clone https://github.com/ros-planning/navigation2.git --branch $ROS_DISTRO ./src/navigation2
  vcs import ./src < ./src/navigation2/tools/underlay.repos
  rosdep install -y \
    --from-paths ./src \
    --ignore-src
  colcon build \
    --symlink-install

You can then ``source $NAV2_WS/install/setup.bash`` to get ready for demonstrations!

.. hint::
  For more examples on building Nav2 from rolling development source, checkout `source.Dockerfile <https://github.com/ros-planning/navigation2/blob/main/tools/source.Dockerfile>`_.

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
