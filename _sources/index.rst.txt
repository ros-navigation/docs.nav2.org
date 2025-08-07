.. _documentation_home:

*****
|LPN|
*****

.. raw:: html

    <h1 align="center">
      <div>
        <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
          <iframe width="450" height="300" src="https://www.youtube.com/embed/OklxMhdDfe0?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
          <iframe width="450" height="300" src="https://www.youtube.com/embed/CYaN43TJANc?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
        </div>
      </div>
    </h1>

Our Sponsors
############

.. image:: images/sponsors_july_2025.png
    :width: 700px
    :align: center
    :alt: Our Sponsors

Services
########

If you need professional services related to Nav2, please contact Open Navigation at info@opennav.org.

Overview
########

Nav2 is the professionally-supported successor of the ROS Navigation Stack deploying the same kinds of technology powering Autonomous Vehicles brought down, optimized, and reworked for mobile and surface robotics.
This project allows for mobile robots to navigate through complex environments to complete user-defined application tasks with nearly any class of robot kinematics.
Not only can it move from Point A to Point B, but it can have intermediary poses, and represent other types of tasks like object following, complete coverage navigation, and more.
Nav2 is a production-grade and high-quality navigation framework trusted by 100+ companies worldwide.

It provides perception, planning, control, localization, visualization, and much more to build highly reliable autonomous systems.
This will compute an environmental model from sensor and semantic data, dynamically path plan, compute velocities for motors, avoid obstacles, and structure higher-level robot behaviors.
To learn more about this project, such as related projects, robots using, ROS1 comparison, and maintainers, see :ref:`about`.
To learn more about navigation and ROS concepts, see :ref:`concepts`.

Nav2 uses behavior trees to create customized and intelligent navigation behavior via orchestrating many independent modular servers.
A task server can be used to compute a path, control effort, behavior, or any other navigation
related task. These separate servers communicate with the behavior tree (BT)
over a ROS interface such as an action server or service.
A robot may utilize potentially many different behavior trees to allow a robot to perform many types of unique and complex tasks.

The diagram below will give you a good first-look at the structure of Nav2.
Note that it is possible to have multiple plugins for controllers, planners,
and recoveries in each of their servers. This can be used to create contextual navigation behaviors.
Each of the servers also returns status indicators back to the BT Navigator in order to enact contextual behaviors based on their results.

The expected inputs to Nav2 are TF transformations conforming to REP-105, a
map source if utilizing the Static Costmap Layer, a BT XML file, and any relevant sensor data
sources. It will then provide valid velocity commands for the motors of a holonomic or
non-holonomic robot to follow when properly configured. We currently support all of the major robot types:
holonomic, differential-drive, legged, and ackermann (car-like) base types! We support
them uniquely with both circular and arbitrarily-shaped robots for SE2 collision checking.


It has tools to:

- Load, serve, and store maps
- Localize the robot on a provided map (SLAM provides the initial map)
- Plan a complete path through the environment, even kinematically feasibly for large robots
- Control the robot to follows the path and dynamically adjust to avoid collision
- Smooth plans to be more continuous, smooth, and/or feasible
- Convert sensor data into an environmental model of the world
- Build complicated and highly-customizable robot behaviors using behavior trees
- Conduct pre-defined behaviors in case of failure, human intervention, or other
- Follow sequential waypoints comprising a mission
- Manage the system's program lifecycle and watchdog for the servers
- Easy dynamically loaded plugins for creating customized algorithms, behaviors and so on
- Monitor raw sensor data for imminent collision or dangerous situation
- Python3 API to interact with Nav2 and its internal task servers in a pythonic manner
- A smoother on output velocities to guarantee dynamic feasibility of commands
- ... and more!

.. image:: images/nav2_architecture.png
    :width: 700px
    :align: center
    :alt: Navigation2 Block Diagram

We also provide a set of starting plugins to get you going.
A list of all plugins can be found on :ref:`plugins` - but they include algorithms for the spanning cross section of common behaviors and robot platform types.

Distributions
#############

Nav2 is available across multiple ROS 2 distributions with varying levels of support:

.. list-table::
   :widths: 50 50
   :header-rows: 0
   :class: distribution-table
   :align: center

   * - .. raw:: html

          <div style="text-align: center;">
            <div style="margin-bottom: 20px;"><span style="font-size: 20px; font-weight: bold;">Rolling Ridley</span></div>
            <div style="margin-bottom: 20px;"><span style="background-color: #007bff; color: white; padding: 12px 24px; border-radius: 25px; font-size: 15px; font-weight: bold;">Development</span></div>
          </div>

       .. image:: distro_graphics/rolling.png
          :width: 300px
          :height: 300px
          :align: center
     - .. raw:: html

          <div style="text-align: center;">
            <div style="margin-bottom: 20px;"><span style="font-size: 20px; font-weight: bold;">Kilted Kaiju</span></div>
            <div style="margin-bottom: 20px;"><span style="background-color: #28a745; color: white; padding: 12px 24px; border-radius: 25px; font-size: 15px; font-weight: bold;">Active Support</span></div>
          </div>

       .. image:: distro_graphics/kilted.png
          :width: 300px
          :height: 300px
          :align: center
   * - .. raw:: html

          <div style="text-align: center;">
            <div style="margin-bottom: 20px;"><span style="font-size: 20px; font-weight: bold;">Jazzy Jalapa</span></div>
            <div style="margin-bottom: 20px;"><span style="background-color: #28a745; color: white; padding: 12px 24px; border-radius: 25px; font-size: 15px; font-weight: bold;">Active Support</span></div>
          </div>

       .. image:: distro_graphics/Jazzy.png
          :width: 300px
          :height: 300px
          :align: center
     - .. raw:: html

          <div style="text-align: center;">
            <div style="margin-bottom: 20px;"><span style="font-size: 20px; font-weight: bold;">Iron Irwini</span></div>
            <div style="margin-bottom: 20px;"><span style="background-color: #dc3545; color: white; padding: 12px 24px; border-radius: 25px; font-size: 15px; font-weight: bold;">End of Life</span></div>
          </div>

       .. image:: distro_graphics/iron.png
          :width: 300px
          :height: 300px
          :align: center
   * - .. raw:: html

          <div style="text-align: center;">
            <div style="margin-bottom: 20px;"><span style="font-size: 20px; font-weight: bold;">Humble Hawksbill</span></div>
            <div style="margin-bottom: 20px;"><span style="background-color: #ffc107; color: black; padding: 12px 24px; border-radius: 25px; font-size: 15px; font-weight: bold;">Maintained</span></div>
          </div>

       .. image:: distro_graphics/humble.png
          :width: 300px
          :height: 300px
          :align: center
     - .. raw:: html

          <div style="text-align: center;">
            <div style="margin-bottom: 20px;"><span style="font-size: 20px; font-weight: bold;">Galactic Geochelone</span></div>
            <div style="margin-bottom: 20px;"><span style="background-color: #dc3545; color: white; padding: 12px 24px; border-radius: 25px; font-size: 15px; font-weight: bold;">End of Life</span></div>
          </div>

       .. image:: distro_graphics/galactic.png
          :width: 300px
          :height: 300px
          :align: center

Related Projects
################

Check out related projects to Nav2:

- `BehaviorTree.CPP <https://www.behaviortree.dev/>`_
- `Robot Localization <https://github.com/cra-ros-pkg/robot_localization>`_
- `SLAM Toolbox <https://github.com/SteveMacenski/slam_toolbox>`_
- `Fuse <https://github.com/locusrobotics/fuse>`_

.. toctree::
   :hidden:

   getting_started/index.rst
   development_guides/index.rst
   concepts/index.rst
   setup_guides/index.rst
   about/robots.rst
   about/roscon.rst
   tutorials/index.rst
   plugin_tutorials/index.rst
   configuration/index.rst
   tuning/index.rst
   behavior_trees/index.rst
   plugins/index.rst
   migration/index.rst
   commander_api/index.rst
   API Docs <https://api.nav2.org>
   roadmap/roadmap.rst
   citations.rst
   about/index.rst
   maintainer_docs/index.rst
