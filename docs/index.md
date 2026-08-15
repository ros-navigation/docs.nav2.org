---
expand_sidebar_nav: true
edit_uri: https://github.com/ros-navigation/mkdocs.nav2.org/tree/rolling/docs/
---

# **NAV2** { #nav2 }

## Your Autonomous Navigation Framework

An open-source, production-grade autonomy framework trusted by 300+ companies worldwide. Building on 15 years of heritage to accelerate the robotics industry.

<div class="video-container">
  <video autoplay muted loop playsinline>
    <source src="assets/hero_video.mp4" type="video/mp4">
  </video>
</div>

[Get Started :octicons-arrow-right-16:][getting-started]{ .md-button .md-button--primary }
[Concepts :octicons-light-bulb-16:][navigation-concepts]{ .md-button }
[First-Time Setup :octicons-tools-16:][first-time-robot-setup-guide]{ .md-button }

## **Our Sponsors**

<figure markdown="span">
  ![Our Sponsors](assets/sponsors_oct_2025.png){ width="700px" title="Our Sponsors"}
</figure>

## **Services**

If you need professional services related to Nav2, please contact Open Navigation at [info@opennav.org](mailto:info@opennav.org).

## **Overview**

Nav2 is the professionally-supported successor of the ROS Navigation Stack deploying the same kinds of technology powering Autonomous Vehicles brought down, optimized, and reworked for mobile and surface robotics.
This project allows for mobile robots to navigate through complex environments to complete user-defined application tasks with nearly any class of robot kinematics and dynamics; shape size; indoor or outdoor, or sensor configuration.
Not only can it move from Point A to Point B, but it can have intermediary poses, and represent other types of tasks like object following, complete coverage navigation, and more.
Nav2 is a production-grade and high-quality navigation framework trusted by 300+ companies worldwide.

It provides perception, planning, control, localization, visualization, behaviors, and much more to build highly reliable autonomous systems.
This will compute an environmental model from sensor and semantic data, dynamically route a path through the environment, compute feasible motor commands, avoid obstacles, and structures higher-level robot behaviors.

Nav2 uses behavior trees to create customized and intelligent navigation behavior via orchestrating many independent modular servers.
A task server can be used to compute a path, control effort, behavior, or any other navigation
related task. These separate servers communicate with the behavior tree (BT)
over a ROS interface such as an action server or service.
A robot may utilize potentially many different behavior trees to allow a robot to perform many types of unique and complex tasks.
A task server may have multiple plugins for controllers, planners, and behaviors to create contextual navigation behaviors.

<!-- It has tools to:

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
- ... and more! -->

<figure markdown="span">
  ![Navigation2 Block Diagram](assets/nav2_architecture.png){ width="700px" title="Navigation2 Block Diagram"}
</figure>

We also provide a set of starting plugins to get you going.
A list of all plugins can be found on [Navigation Plugins][navigation-plugins] - but they include algorithms for the spanning cross section of common behaviors and robot platform types.

<div class="grid cards" markdown>

- :octicons-book-16: [Citations][citations] - If you use the navigation framework, an algorithm from this repository, or ideas from it please cite this work in your papers!

</div>

<!-- CSS located in overrides/assets/stylesheets/robots_marquee.css -->
<div class="robots-marquee">
  <div class="robots-marquee-track">
    <a href="robots_using/"><img src="robots_using/assets/dexory.png" alt="Dexory"></a>
    <a href="robots_using/"><img src="robots_using/assets/novacarter.png" alt="Nvidia Nova Carter"></a>
    <a href="robots_using/"><img src="robots_using/assets/kiwibot.png" alt="Kiwibot"></a>
    <a href="robots_using/"><img src="robots_using/assets/firefly.png" alt="Firefly Automatix"></a>
    <a href="robots_using/"><img src="robots_using/assets/g1.png" alt="Unitree G1"></a>
    <a href="robots_using/"><img src="robots_using/assets/go2.png" alt="Unitree Go2"></a>
    <a href="robots_using/"><img src="robots_using/assets/angsa.png" alt="Angsa"></a>
    <a href="robots_using/"><img src="robots_using/assets/polymath.png" alt="Polymath Robotics"></a>
    <a href="robots_using/"><img src="robots_using/assets/karelics2.png" alt="Karelics"></a>
    <a href="robots_using/"><img src="robots_using/assets/tailos.png" alt="Rosie"></a>
    <a href="robots_using/"><img src="robots_using/assets/seasony.png" alt="Seasony"></a>
    <a href="robots_using/"><img src="robots_using/assets/barnowl.png" alt="Barn Owl"></a>
    <a href="robots_using/"><img src="robots_using/assets/tb4.png" alt="Turtlebot4"></a>
    <a href="robots_using/"><img src="robots_using/assets/tri.png" alt="Toyota"></a>
    <a href="robots_using/"><img src="robots_using/assets/elroy.png" alt="Elroy Air"></a>
    <a href="robots_using/"><img src="robots_using/assets/torch.png" alt="Torch Technologies"></a>
    <a href="robots_using/"><img src="robots_using/assets/botronics.png" alt="Botronics"></a>
    <!-- Duplicate for seamless loop -->
    <a href="robots_using/"><img src="robots_using/assets/dexory.png" alt="Dexory"></a>
    <a href="robots_using/"><img src="robots_using/assets/novacarter.png" alt="Nvidia Nova Carter"></a>
    <a href="robots_using/"><img src="robots_using/assets/kiwibot.png" alt="Kiwibot"></a>
    <a href="robots_using/"><img src="robots_using/assets/firefly.png" alt="Firefly Automatix"></a>
    <a href="robots_using/"><img src="robots_using/assets/g1.png" alt="Unitree G1"></a>
    <a href="robots_using/"><img src="robots_using/assets/go2.png" alt="Unitree Go2"></a>
    <a href="robots_using/"><img src="robots_using/assets/angsa.png" alt="Angsa"></a>
    <a href="robots_using/"><img src="robots_using/assets/polymath.png" alt="Polymath Robotics"></a>
    <a href="robots_using/"><img src="robots_using/assets/karelics2.png" alt="Karelics"></a>
    <a href="robots_using/"><img src="robots_using/assets/tailos.png" alt="Rosie"></a>
    <a href="robots_using/"><img src="robots_using/assets/seasony.png" alt="Seasony"></a>
    <a href="robots_using/"><img src="robots_using/assets/barnowl.png" alt="Barn Owl"></a>
    <a href="robots_using/"><img src="robots_using/assets/tb4.png" alt="Turtlebot4"></a>
    <a href="robots_using/"><img src="robots_using/assets/tri.png" alt="Toyota"></a>
    <a href="robots_using/"><img src="robots_using/assets/elroy.png" alt="Elroy Air"></a>
    <a href="robots_using/"><img src="robots_using/assets/torch.png" alt="Torch Technologies"></a>
    <a href="robots_using/"><img src="robots_using/assets/botronics.png" alt="Botronics"></a>
  </div>
</div>


## **Distributions**

Nav2 is available across multiple ROS 2 distributions with varying levels of support:

<!-- CSS files located in overrides/assets/stylesheets/distro_grid.css -->
<div class="distro-grid">
  <div class="distro-cell">
    <div class="distro-title">Rolling Ridley</div>
    <div class="distro-badge dev">Development</div>
    <img src="assets/distro_graphics/rolling.png"/>
  </div>
  <div class="distro-cell">
    <div class="distro-title">Lyrical Lynx</div>
    <div class="distro-badge active">Active Support</div>
    <img src="assets/distro_graphics/lyrical.png"/>
  </div>
  <div class="distro-cell">
    <div class="distro-title">Kilted Kaiju</div>
    <div class="distro-badge maintained">Maintained</div>
    <img src="assets/distro_graphics/kilted.png"/>
  </div>
  <div class="distro-cell">
    <div class="distro-title">Jazzy Jalisco</div>
    <div class="distro-badge active">Active Support</div>
    <img src="assets/distro_graphics/jazzy.png"/>
  </div>
  <div class="distro-cell">
    <div class="distro-title">Iron Irwini</div>
    <div class="distro-badge eol">End of Life</div>
    <img src="assets/distro_graphics/iron.png"/>
  </div>
  <div class="distro-cell">
    <div class="distro-title">Humble Hawksbill</div>
    <div class="distro-badge maintained">Maintained</div>
    <img src="assets/distro_graphics/humble.png"/>
  </div>
  <div class="distro-cell">
    <div class="distro-title">Galactic Geochelone</div>
    <div class="distro-badge eol">End of Life</div>
    <img src="assets/distro_graphics/galactic.png"/>
  </div>
</div>
