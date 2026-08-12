# Collision Monitor { #collision-monitor }

Source code and `README` with design, explanations, and metrics can be found on [Github](https://github.com/ros-navigation/navigation2/tree/main/nav2_collision_monitor).

The `nav2_collision_monitor` package contains nodes providing an additional level of robot safety, namely the Collision Monitor and the Collision Detector.
The Collision Monitor is a node providing an additional level of robot safety. It performs several collision avoidance related tasks using incoming data from the sensors, bypassing the costmap and trajectory planners, to monitor for and prevent potential collisions at the emergency-stop level.
The Collision Detector works similarly to the Collision Monitor, but does not affect the robot's velocity. It will only inform that data from the configured sources has been detected within the configured polygons via a message to a topic.

## Provided Nodes

The nodes listed below are inside the `nav2_collision_monitor` package. See the pages for individual configuration information.

<div class="grid cards bottom-align" markdown>

- :material-shield-alert: **Collision Monitor Node**

    ---
    Detects and prevents imminent collisions by limiting velocity commands.

    [:octicons-arrow-right-24: Go][collision-monitor-node]

- :material-shield-search: **Collision Detector Node**

    ---
    Reports detected collisions without modifying velocity commands.

    [:octicons-arrow-right-24: Go][collision-detector-node]

</div>
