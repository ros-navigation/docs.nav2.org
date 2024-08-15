.. _docker_development:

Docker for Development: Zero to Hero
************************************

- `Overview`_
- `Preliminaries`_
- `Important Docker Commands`_
- `Exploring Your First Container`_
- `Understanding ROS Docker Images`_
- `For Docker-Based Development`_
- `For Docker-Based Deployment`_
- `Conclusion`_
- `Appendix`_

Overview
========

This tutorial is a hands on walkthrough from your first Docker usage to commands and processes you can use for development of ROS 2 applications using Docker.
This is not an exhaustive introduction by any stretch, but should help you go from nothing to a functional process you can use on a daily basis for development and testing.
If you know some basics about Docker already, you can skip to later sections about development and deployment. 
You'll also find a set of Docker images in the Appendix to this tutorial which are useful for developing with Nav2 or a containerized deployment of it.
The same process template can be used for other company applications and projects as well.

Some other useful resources:

- `Articulated Robotics <https://www.youtube.com/watch?v=XcJzOYe3E6M>`_ 4-part series on Docker covering different detail and focuses in video format.
- `Docker Documentation <https://docs.docker.com/>`_ for comprehensive information
- `OSRF DockerHub <https://hub.docker.com/_/ros/>`_ containing docker images for various ROS distributions and install packages and its `source images <https://github.com/osrf/docker_images/tree/master/ros>`_.

Preliminaries
=============

Docker is a tool used to build, deploy, test, and otherwise work with software in an isolated environment, called a *container*.
This diffs from a VM in that it shares the same linux kernel as your host operating system, making it faster to spin up and share host resources.
By building or deploying software in this isolated environment, you can ensure many users, robots, or servers are running the same software with the same software versions across many instances.
It gives you a controlled environment to work in that is reproducible on other developer's machines and even work in a different (linux-based) operating system than your computer currently runs.
For example, you can run a 22.04 Docker container that contains ROS 2 Humble on Nvidia Jetson's Jetpack 5.1 (which is a flavor of 20.04) and deploy that container to a fleet of robots.

In common Docker language, an *image* is the built ``Dockerfile`` which can be used to create *containers*.
Thus, *containers* are self-contained, runnable instances of the docker *image*. 
The ``Dockerfile`` is a set of instructions for how to build an image to create some kind of environment to work in - and often contain applications to deploy in that environment.
The Dockerfile instruct sets have a number of options such as:

- ``ARG``: Obtain build-time arguments
- ``FROM``: Specify a base image to build from
- ``RUN``: Run a particular command
- ``WORKDIR``: Set the working directory
- ``COPY``: Copy a file or directory
- ``ENV``: Set an environmental variable

Most of these are self explanatory, but you can reference the Docker documentation to learn more and see the full set.

Two special commands worth highlighting are ``CMD`` and ``ENTRYPOINT`` which you will see at the bottom of many ``Dockerfile`` s.

- ``ENTRYPOINT``: A command to execute when the container is spun up which cannot be overridden
- ``CMD``: A command to execute when a container is spun up which can be overridden

In the context of ROS Docker containers, you'll see that these create a bash session and execute a ``ros_entrypoint.sh`` script.
This script simply sources the ROS environment ``/opt/ros/.../setup.bash`` for your distribution so when you open the container, you're ready to go.
But these can be used to do more advanced things like run your application or trigger other events.

Important Docker Commands
=========================

Similarly unexhaustively, it is important to discuss a few docker commandline commands before we move forward. 
There are many others, but these are the basic commands which we will use in this walkthrough and likely the most common you'll use on the day-to-day.
Over the tutorial, build up some important option flags for each, but for now lets talk about the basics:

- ``docker run``: Runs a given docker image to create a container
- ``docker build``: Builds a Dockerfile to create an image
- ``docker pull / push``: Pulls a docker image from another location or pushes a built image to another location
- ``docker stop / kill``: Stops or kills a running docker container 
- ``docker ps``: Lists a set of docker images that are currently running
- ``docker attach``: Attach a terminal to a background running docker container
- ``docker exec``: Execute a command in a provided container
- ``docker images``: Lists a set of containers pulled or built on your computer

Exploring Your First Container
==============================

Lets start this tutorial with getting the latest-and-greatest of ROS 2 Rolling.
Using the OSRF DockerHub server, we can pull down a number of different ROS 2 docker images to work with without having to create a Dockerfile or build them ourselves. 

.. code-block:: bash

  sudo docker pull osrf/ros:rolling-desktop-full

You should then see the following, where the image is being pulled down in multiple layers and finally returning the terminal once complete.

.. code-block:: bash

  steve@reese:~$ sudo docker pull osrf/ros:rolling-desktop-full
  rolling-desktop-full: Pulling from osrf/ros
  31bd5f451a84: Already exists 
  d36cae3fb404: Already exists 
  8d68f36a56a7: Already exists 
  299f725c4bf1: Already exists 
  6e16227afc48: Already exists 
  02457a85146c: Downloading   83.7MB/106.5MB
  fe0cbdee2808: Download complete 
  4b4dbddf506a: Downloading  92.86MB/98.14MB
  0da90b52c355: Download complete 
  64de492566b2: Download complete 
  167d95ac0fce: Download complete 
  e727072615d0: Downloading  82.61MB/809.8MB
  d15e176ed0af: Waiting 

If you then attempt to run this image as a container (the instance of the image):

.. code-block:: bash

  sudo docker run osrf/ros:rolling-desktop-full

You should see that it runs for a second then exits the terminal. Yay! It works! But... that's not very useful, now is it? 
Our ``ENTRYPOINT`` for the ROS 2 Docker images only sources the ROS 2 installation and so the program returns as completed.
If we want to get into the container to do something useful for ourselves in that environment, we need to open an interactive terminal session with the container.
This is easy to do with the ``-it`` flags:

.. code-block:: bash

  sudo docker run -it osrf/ros:rolling-desktop-full

You should now see a terminal session open with a command prompt of ``root@<some hash>:/#``.
This is your docker container. 
Take a look around, it should look like any other linux OS.
If you go into ``/opt/ros/rolling``, it should look familiar to you!

------------

If you open a new terminal and run ``sudo docker ps``, you should see now one container instance running on your system.
The ID of this container should match the hash in your command prompt.
We mentioned before that the container on spin up will automatically source the ROS installation, so we should be able to immediately play around:

.. code-block:: bash

  echo $ROS_DISTRO  # --> rolling
  ros2 run demo_nodes_cpp talker # --> [INFO] [1707513434.798456374] [talker]: Publishing: 'Hello World: 1'
  touch navigator_dockerlayer.txt
  l # <-- you should see this file

Nice! It all works. Now, if we exit our interactive session (type ``exit``), we should reenter our computer.
In that second terminal, if you rerun ``sudo docker ps``, you should see that the list of containers is now empty since our container is no longer running.
If you want to see a full list of containers, including exited containers, you can use the flag ``-a`` to display all containers.

.. code-block:: bash

  steve@reese:~$ sudo docker ps -a
  CONTAINER ID   IMAGE                           COMMAND                  CREATED         STATUS                          PORTS     NAMES
  7ec0e0b7487f   osrf/ros:rolling-desktop-full   "/ros_entrypoint.sh …"   5 minutes ago   Exited (0) About a minute ago             strange_tesla
  9ccd97ac14f9   osrf/ros:rolling-desktop-full   "/ros_entrypoint.sh …"   7 minutes ago   Exited (0) 7 minutes ago                  zen_perlman

You can see that our container exited successfully. If we now run our docker image again, you should see it back listed without ``-a``.

.. code-block:: bash

  sudo docker run -it osrf/ros:rolling-desktop-full

While we're here, lets ``ls`` our container. Oh no! Our ``navigator_dockerlayer.txt`` file is missing!
That's completely to be expected. When we exit the container, that instance of the image is destroyed - never again to be seen.
When we run the image again, we're generating a brand new, clean instance of the image.
Nothing persists. This is an important behavior to understand moving forward. 
For development, this is nightmare fuel of losing a day's work by hitting the wrong button. 
For deployment, this is a blessing as you can cleanly restart with no artifacts from a previously failed session and start with a clean slate.
We'll discuss how to persist data between sessions later on in the tutorial, so fear not!

------------

With our new container still open, lets explore how to work with one container across multiple terminals. If you were to run the ``docker run`` command in two terminals, you'd make two separate containers isolated from each other.
Instead, we need to open a new session in the container. Looking at the terminal's hash or ``sudo docker ps`` to find its ID, use the ``exec`` command to execute the command ``bash`` in the container.

.. code-block:: bash

  sudo docker exec -it bce2ad161bf7 bash  # <-- use your ID

This opens a new interactive session to the container and ``exec`` -utes the command ``bash`` to give us a shell to work with (``CMD`` in our Dockerfile does this for us for the spin up terminal).
Since this isn't a newly spun up container, the ``ENTRYPOINT`` script wasn't run. If you try to run the talker demo again, it won't find the ``ros2`` command.
Fear not, simply source your ``/opt/ros/rolling/setup.bash`` install and you're good to go.

In either terminal session in the container, if you create a new file, you should be able to see it in the other since this is the same container!

.. code-block:: bash

  touch navigator_alligator.txt
  ls # <-- see the new file
  # move to the other terminal
  ls # <-- also see new file

Now we can do something fun while we have both terminals of the same docker container open. Lets run the classic talker/listener demo. In each of the two terminals, run one of these commands.

.. code-block:: bash

	ros2 run ros2 run demo_nodes_cpp talker
	ros2 run demo_nodes_py listener

------------

If you now open a third terminal to your computer and run ``ros2 topic list``, you'll see a notable lack of topics.

.. code-block:: bash

  steve@reese:~$ ros2 topic list 
  /parameter_events
  /rosout

What gives? The container is isolated from your host system, so anything happening in the container is currently unavailable to your main computer.
Lets exit our two container terminal instances (``exit``) and talk about some more ``docker run`` flags that are useful to know.
This time, we want to expose ROS to our broader system, including our host computer. This time, we'll use the flag ``--net=host``, this sets the network to look like the host system (i.e. your computer).

.. code-block:: bash

	sudo docker run -it --net=host osrf/ros:rolling-desktop-full

In this session, if we run the talker ``ros2 run demo_nodes_py talker``, now we should be able to subscribe to it from our host computer!

.. code-block:: bash

  steve@reese:~$ ros2 topic echo /chatter
  data: 'Hello World: 0'
  ---
  data: 'Hello World: 1'
  ---
  data: 'Hello World: 2'
  ---

------------

Lets talk about how to keep a container running for longer than than your interactive terminal session.
There are many reasons you want a container to outlive you or run in the background, so that's what the ``-d`` flag is for, or detached.
Lets start off by showing that there are no containers running with ``sudo docker ps``. Next start a new container with the flag.

.. code-block:: bash

	sudo docker run -it --net=host -d osrf/ros:rolling-desktop-full

You'll see the command run for a moment and return. ``sudo docker ps`` should now show a container running.
Copy that container ID and we can now ``attach`` to it:

.. code-block:: bash

	sudo docker attach e1d7e035a824  # <-- use your ID

You should now be in the terminal session. After you do your work, if you want to stop the container, you can exit as we have been in this tutorial (``exit``) and that will also stop the container.
If you wish to leave the container running, you can use the key sequence Control+P+Q to exit but leave the container running.
In either case, you can show that to yourself using ``ps``.
If you left it running and now wish to stop it externally, you can do so with the following. It may take a few moments to exit.

.. code-block:: bash

	sudo docker stop e1d7e035a824  # <-- use your ID

------------

Finally, ``docker images`` is a command used to tell you what docker images you have built or pulled which are available for use. This list will expand over time and is a useful resource to see what you have to work with.

.. code-block:: bash

  steve@reese:~$ sudo docker images
  REPOSITORY   TAG                    IMAGE ID       CREATED        SIZE
  osrf/ros     rolling-desktop-full   7cd0c5068235   6 days ago     3.86GB

.. note:: If errors are seen *Failed to create Shared Memory Manager* or similar, use the ``--shm-size=100mb`` command to increase the shared memory buffer size in the container.

Understanding ROS Docker Images
===============================

Now that we know a bit about Docker's basic features and explored the Rolling Desktop Full container, lets look at the Docker images you have to work with in ROS in more detail.
OSRF hosts a DockerHub server containing images of all ROS distributions which you can pull and use.
For each distribution, there are a couple of variants: 

- ``ros-core``: Contains only the ROS core communication protocols and utilities
- ``ros-base``: Contains ``ros-core`` and other core utilities like pluginlib, bond, actions, etc
- ``perception``: Contains ``ros-base`` and image common, pipeline, laser filters, laser geometry, vision opencv, etc
- ``desktop``: Contains ``ros-base`` and tutorials, lifecycle, rviz2, teleop, and rqt
- ``desktop-full``: Contains ``desktop``, ``perception`` and simulation

These are the same as if you were to use `apt install ros-rolling-desktop-full`, but in container form.
Each of those containers build off of the previous one using ``FROM`` and then install the binaries described to serve to the container user.
Which you use depends on your application and needs, but ``osrf/ros:<distro>-ros-base`` is a good default for development and deployment.
We're using desktop-full in the context of this tutorial for ease of having rviz2 and such built-in batteries-included.

You can pull and use them the same way as before, for example:

.. code-block:: bash

  sudo docker pull ros:rolling-ros-base
  sudo docker pull osrf/ros:humble-desktop

Note that some containers may require ``osrf/`` and others may not. The ``osrf/`` images are released by osrf while the non-prefixed are a part of the official docker libraries.
In general, the desktop installs are with ``osrf/``` and the ros core and base are without.

For Docker-Based Development
============================

As mentioned previously, if we create and modify files in the Docker container, these do not persist after the container is exited.
If we want to do some development work that will persist between images, it is wise to *mount* a *volume* to the docker container when we run it.
That is just fancy talk for linking a given set of directories from your host company to the container so that they can be read, modified, and deleted within the container and reflected on the outside.
That way, your work will persist even if you close a container in your local filesystem as if it were developed without the use of a container.
An awesome feature of this is that you can actually build your workspace in one container, destroy that container, and then continue development and rebuild in a new container instance later provided that (1) the same image is used both times and (2) the mounted location within the container is the same each time.

We accomplish this using the ``-v`` flag (for volume). There are other options to do this as well, but this is the most straight forward.
It takes in the argument in the form ``-v what/local/dir:/absolute/path/in/container``.
If we start a container in our workspace's root, the following will launch the docker container, sharing the host's network, and putting your workspace (``.``) into the container under the directory ``/my_ws_docker``:

.. code-block:: bash

  sudo docker run -it --net=host -v .:/my_ws_docker  osrf/ros:rolling-desktop-full

  ls
  cd my_ws_docker
  touch navigator_activator.txt

If you go to your workspace in another terminal, you should now see that file reflected on your computer! If we run rosdep to install our dependencies in the docker container, we should now be able to build your workspace.

.. code-block:: bash

  apt update
  rosdep init
  rosdep update
  rosdep install -r -y --from-paths . --ignore-src
  colcon build

Now, you can make any changes to your code using VSCode or your favorite code editor and have it reflected in the container for building and testing! 
This is especially powerful if you're working with multiple ROS distributions or with a ROS distribution which your host OS doesn't natively support (such as Humble on Jetpack 5.1 on Nvidia Jetsons).
However, it does get annoying over time to have to wait for all of your dependencies to install manually when you spin up a new container.
Thus, it is useful to build atop one of the provided ROS Docker images to create your own custom development image containing the packages and environment you need to build your application.
That way, you can simply jump into the container and immediately start building.

Building a Development Image
----------------------------

Building a new container is easy. The organization instructions of Docker images are outlined in ``Dockerfile`` s.
Typically, they start with an import ``FROM`` to set the starting container to build off of. In our case, a ROS 2 Rolling image.
Then, we run a series of ``RUN`` commands to perform actions to setup our dependencies so we can have them ready for use when we launch a container.
In the ``Appendix``, you'll find an example development image that you can use to develop on Nav2. It starts with Rolling ``ros-base``, downloads Nav2, and runs rosdep over its packages to install all dependencies.
Once these steps conclude, the image is all setup for any later Nav2 build.

You can build this image using ``docker build``

.. code-block:: bash

  sudo docker build -t nav2deps:rolling .

Where ``-t`` sets the tagged name of the container for later use.
Its important to note that even though your install and build spaces will be reflected in your host workspace, they cannot be run locally when compiled inside of a docker container.
This example development image also upgrades packages which breaks strict version controlling of system and ``ros-base`` installed packages.
For a deployment situation, you want to ensure you have the same version of all packages -- however for ROS 2 Rolling where ABI and API are not promised to be stable due to live development, 
it is useful to upgrade so that your source code can build against the latest and greatest.

Visualizations from Docker
--------------------------

Some that skip ahead at this point might notice that when launching their applications which involve a GUI (RQT, Rviz2, Gazebo), it crashes and never appears.
Docker's isolation isn't just for networking, but also in visualization and other assets.
Thus, we must specifically enable carve outs for GUIs to appear on our screens.

- ``--privileged``: Bypasses many of the checks to field the container from the host system. A hammer smashing isolation.
- ``--env="DISPLAY=$DISPLAY``: Sets display to use for GUI
- ``--volume="${XAUTHORITY}:/root/.Xauthority"``: Gets important info from the XServer for graphics display

Putting it altogether, you should now be able to open rviz2 inside of the docker container!

.. code-block:: bash

	sudo docker run -it --net=host --privileged \
	    --env="DISPLAY=$DISPLAY" \
	    --volume="${XAUTHORITY}:/root/.Xauthority" \
	    osrf/ros:rolling-desktop-full

.. code-block:: bash

  rviz2

At this point, if you have an error remaining, please check docs for the right flags to use.
(Even if you copy+paste around, it shouldn't take you more than 10 minutes to find a combo that works.)
If you're on Nvidia Jetson hardware, reference their documentation for the correct set of flags for your Jetpack version.

For Docker-Based Deployment
===========================

We won't belabor the details, but Docker is not just for development, but for application deployment as well.
You can run instances of your image on robots, cloud servers, etc as self-containing micro-services or robot application systems.

Typically speaking, you would set your ``ENTRYPOINT`` to launch a script which brings up and runs your server(s) for your application.
For example, you could use the deployment image in the ``Appendix`` with an ``ENTRYPOINT`` to launch your root robot navigation launch file ``tb3_simulation_gazebo_launch.py``, or similar.
You could even have the container launch on bringup using ``systemd`` in order to have your application automatically launch, containerized, on system startup.

Conclusion
==========

At the end of this, you should be able to now:

- Pull the official ROS 2 docker images of any ROS distribution and choose the right type of image for your needs
- Understand how ROS 2 docker containers are formatted and the core part of ``Dockerfile`` image descriptions
- Understand Docker's filesystem and network isolation -- and how to bypass it for important use-cases in development
- Be able to detach your docker containers for long-running processes 
- Mount your development workspace to the container to work in
- Build your own docker image off of ROS' for your development dependencies and setup needs
- Visualization and simulation with GUI in docker

Its useful to note at this point that the ``--privileged`` flag is a real hammer. If you want to avoid running this, you can find all the individual areas you need to enable for visualization to work.
Also note that ``--privileged`` also makes it easier to run hardware interfaces like joysticks and sensors by enabling inputs from the host operating system that are processing those inputs.
If in production, you cannot use a hammer, you may need to dig into your system a bit to allow through only the interfaces required for your hardware.

As for potential steps forward: 

- Setup a config file to hide all those docker run arguments for development
- Setup a bash script to enable several different configurations of docker run and execute the run itself
- Learn more about Docker's options and features such as `compose <https://docs.docker.com/compose/>`_, pushing your own containers to DockerHub, and version controlling images
- Limit and regulate host resource utilization
- `Configure computer <https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_ to avoid use of ``sudo`` for each docker CLI command.
- Consider production considerations like build cache management, security, multi-stage builds, and such to get the most out of Docker

We hope that's enough to get you started! 

-- Your Friendly Neighborhood Navigators

Appendix
========

Nav2 Development Image
----------------------

This container downloads, but does not install Nav2.
Instead, it pulls the dependencies so that when you run this container, you obtain everything needed to immediately start building and working with Nav2 on any ROS 2 distribution, including Rolling.

.. code-block:: bash

  ARG ROS_DISTRO=rolling
  FROM ros:${ROS_DISTRO}-ros-core

  RUN apt update \
      && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
    ros-dev-tools \
    wget
    
  WORKDIR /root/nav2_ws 
  RUN mkdir -p ~/nav2_ws/src
  RUN git clone https://github.com/ros-planning/navigation2.git --branch main ./src/navigation2
  RUN rosdep init
  RUN apt update && apt upgrade -y \
      && rosdep update \
      && rosdep install -y --ignore-src --from-paths src -r

Nav2 Deployment Image
---------------------

This image either downloads and installs Nav2 (Rolling; from source) or installs it (from binaries) to have a self contained image of everything you need to run Nav2.
From here, you can go to the :ref:`getting_started` to test it out! 

.. code-block:: bash

  ARG ROS_DISTRO=rolling
  FROM ros:${ROS_DISTRO}-ros-core

  RUN apt update \
      && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
    ros-dev-tools \
    wget

  # For Rolling or want to build from source a particular branch / fork
  WORKDIR /root/nav2_ws 
  RUN mkdir -p ~/nav2_ws/src
  RUN git clone https://github.com/ros-planning/navigation2.git --branch main ./src/navigation2
  RUN rosdep init
  RUN apt update && apt upgrade -y \
      && rosdep update \
      && rosdep install -y --ignore-src --from-paths src -r
  RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
      && colcon build --symlink-install

  # For all else, uncomment the above Rolling lines and replace with below
  # RUN rosdep init
  # RUN apt update && apt upgrade -y \
  #     && rosdep update \
  #     && apt install \
  #         ros-${NAV2_BRANCH}-nav2-bringup \
  #         ros-${NAV2_BRANCH}-navigation2 \
  #         ros-${NAV2_BRANCH}-turtlebot3-gazebo
