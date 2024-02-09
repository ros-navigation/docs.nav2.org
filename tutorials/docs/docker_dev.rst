.. _docker_development:

Docker for Development and Deployment: Zero to Hero
***************************************************

- `Overview`_
- `Preliminaries`_
- `Important Docker Commands`_
- `Exploring Your First Container`_
- `Understanding ROS Docker Images`_
- `Exploring ROS Docker and Docker Options`_
- `For Docker-Based Development`_
- `For Docker-Based Deployment`_
- `Conclusions`_
- `Appendix`_

Overview
========

This tutorial is a hands on walkthrough from your first Docker usage to commands and processes you can use for development and deployment of ROS 2 applications using Docker.
This is not an exhausive introduction by any stretch, but should help you go from nothing to a functional process you can use on a daily basis for development and testing.
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
This diffs from a VM in that it shares the same linux kernal as your host operating system, making it faster to spin up and share host resources.
By building or deploying software in this isolated environment, you can ensure many users, robots, or servers are running the same software with the same software versions across many instances.
It gives you a controlled environment to work in that is reproducable on other developer's machines and even work in a different (linux-based) operating system than your computer currently runs.
For example, you can run a 22.04 Docker container on Nvidia Jetson's Jetpack 5.1 (which is a flavor of 20.04) to run ROS 2 Humble and deploy that container to a fleet of robots.

In common Docker language, an *image* is the built ``Dockerfile`` which can be used to create *containers*.
Thus, *containers* are instances of the docker *image*. 
The ``Dockerfile`` is a set of instructions for how to build an image to create some kind of environment to work in - and often contain applications to deploy in that environment.
The Dockerfile instruct sets have a number of options such as:

- ``ARG``: Obtain build-time arugments
- ``FROM``: Specify a base image to build from
- ``RUN``: Run a particular command
- ``WORKDIR``: Set the working directory
- ``COPY``: Copy a file or directory
- ``ENV``: Set an environmental variable

Most of these are self explanatory, but you can reference the Docker documentation to learn more and see the full set.

Two special commands worth highlighting are ``CMD`` and ``ENTRYPOINT`` which you will see at the bottom of many ``Dockerfile``s.

- ``ENTRYPOINT``: A command to execute when the container is spun up which cannot be overrided
- ``CMD``: A command to execute when a container is spun up which can be overrided

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
- ``docker attach``: Attach to a background running docker container
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
This is easy to do with the ``-it`` flag:

.. code-block:: bash

  sudo docker run -it osrf/ros:rolling-desktop-full

You should now see a terminal session open with a command prompt of ``root@<some hash>:/#``.
This is your docker container. 

------------

If you open a new terminal and run ``sudo docker ps``, you should see now one container instance running on your system.
The ID of this container should match the hash in your command prompt.
We mentioned before that the container on spin up will automatically source the ROS installation, so we should be able to immediately play around:

.. code-block:: bash
  echo $ROS_DISTRO  # --> rolling
  ros2 run demo_nodes_cpp talker # --> [INFO] [1707513434.798456374] [talker]: Publishing: 'Hello World: 1'
  touch navigator_dockerlayer.txt
  l # <-- you should see this file

Nice! It all works. Now, if we exit our iteractive session (type ``exit``), we should reenter our computer.
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

This opens a new interactive session to the container and ``exec``utes the command ``bash`` to give us a shell to work with (``CMD`` in our Dockerfile does this for us for the spin up terminal).
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

	sudo docker images

  steve@reese:~$ sudo docker images
  REPOSITORY   TAG                    IMAGE ID       CREATED        SIZE
  osrf/ros     rolling-desktop-full   7cd0c5068235   6 days ago     3.86GB


Understanding ROS Docker Images
===============================

structure, source, etc


Exploring ROS Docker and Docker Options
=======================================


For Docker-Based Development
============================


Building a Development Image
----------------------------


Visualizations from Docker
--------------------------


For Docker-Based Deployment
===========================

Won't belabor with details...




Conclusion
==========

Video? what graphics / images?

So now at the end of this --> You can:
-  pull the ROS 2 docker images and run the demos
- Understand how the ROS 2 docker images are formatted
- Understand Docker's filesystem and network isolation -- and how to bypass it for development and running nodes across your system
- Detach your docker containers for long-running processes 
- Mount your development workspace to the container to work in but persist between container instances
- Build your own docker image off of ROS' for your development dependencies
- Use visualization and simulation with GUI in docker
- and how to deploy software

I hope that's enough to get you started :-) We didn't cover all the options in all the detail, but I think this is functionally enough for almost everyone. 

Steps forward:
- use a config file to hide all those arguments for development. For example setting them all in docker_run.conf, you can do `docker run $(cat docker_run.conf) osrf/ros:rolling-desktop-full
- Use a bash script to set multiple sets of flags for different situations & include the `docker run` bits in it with the container as the argument `db osrf/ros:rolling-desktop-full`
- Learn bout all the other options and features of Docker like compose, dockerhub, and version controlling deployment images
- Adding docker to sudoers group so you dont need to call that every time (sudo usermod -aG docker $USER)
- limit resource utilization

--privledged is a hammer, you can reduce this to more specificity. make sure to take care for hardware inputs


Appendix
========

Nav2 Development Image
----------------------

This container downloads, but does not install Nav2.
Instead, it pulls the dependencies so that when you run this container, you obtain everything needed to immedately start building and working with Nav2 on any ROS 2 distribution, including Rolling.

.. code-block:: bash

  ARG ROS_DISTRO=rolling
  FROM ros:${ROS_DISTRO}-ros-core

  RUN apt-get update \
      && apt-get install -y \
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

  RUN apt-get update \
      && apt-get install -y \
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
  RUN colcon build --symlink-install

  # For all else, uncomment the above Rolling lines and replace with below
  # RUN rosdep init
  # RUN apt update && apt upgrade -y \
  #     && rosdep update \
  #     && apt install ros-rolling-nav2-bringup ros-rolling-navigation2 ros-rolling-turtlebot3-gazebo
