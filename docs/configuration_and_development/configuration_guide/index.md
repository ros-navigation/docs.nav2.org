# Configuration Guide { #configuration-guide }

This guide provides a process through which the user can adjust the tunable parameters to obtain
the best navigation performance.

<span class="section-title">Core Servers</span>

<div class="grid cards bottom-align" markdown>

- :material-graph: **Behavior Server**

    ---
    Primitive behaviors and recovery.

    [:octicons-arrow-right-24: Go][behavior-server]

- :material-file-tree: **Behavior-Tree Navigator**

    ---
    BT execution engine that orchestrates navigation tasks.

    [:octicons-arrow-right-24: Go][behavior-tree-navigator]

- :material-code-tags: **Behavior Tree XML Nodes**

    ---
    Reference for all BT action, condition, control, and decorator plugins.

    [:octicons-arrow-right-24: Go][behavior-tree-xml-nodes]

- :material-shield-alert: **Collision Monitor**

    ---
    Real-time collision monitoring bypassing costmaps.

    [:octicons-arrow-right-24: Go][collision-monitor]

- :material-steering: **Controller Server**

    ---
    Local trajectory planning and velocity command generation.

    [:octicons-arrow-right-24: Go][controller-server]

- :material-grid: **Costmap 2D**

    ---
    2D environmental representation with layered obstacle data.

    [:octicons-arrow-right-24: Go][costmap-2d]

- :material-ev-plug-type2: **Docking Server**

    ---
    Autonomous docking with chargers and static infrastructure.

    [:octicons-arrow-right-24: Go][docking-server]

- :material-state-machine: **Lifecycle Manager**

    ---
    Node lifecycle orchestration (configure, activate, shutdown).

    [:octicons-arrow-right-24: Go][lifecycle-manager]

- :material-map: **Map Server**

    ---
    Static map loading, saving, and serving.

    [:octicons-arrow-right-24: Go][map-server-index]

- :material-map-marker-path: **Planner Server**

    ---
    Global path planning from start to goal.

    [:octicons-arrow-right-24: Go][planner-server]

- :material-transit-connection-variant: **Route Server**

    ---
    Graph-based route planning over waypoint networks.

    [:octicons-arrow-right-24: Go][route-server]

- :material-wave: **Smoother Server**

    ---
    Post-processing path smoothing for feasible trajectories.

    [:octicons-arrow-right-24: Go][smoother-server]

- :material-speedometer-slow: **Velocity Smoother**

    ---
    Smooth velocity commands to respect acceleration limits.

    [:octicons-arrow-right-24: Go][velocity-smoother]

- :material-flag-checkered: **Waypoint Follower**

    ---
    Sequential waypoint execution with per-waypoint task plugins.

    [:octicons-arrow-right-24: Go][waypoint-follower]

</div>

<span class="section-title">Planners Plugins</span>

<div class="grid cards bottom-align" markdown>

- :material-grid: **NavFn Planner**

    ---
    Classic A*/Dijkstra grid-based global planner.

    [:octicons-arrow-right-24: Go][navfn-planner]

- :material-road-variant: **Smac Planner**

    ---
    Family of kinematically-aware planners (2D, Hybrid-A*, Lattice).

    [:octicons-arrow-right-24: Go][smac-planner]

- :material-ray-start-arrow: **Theta Star Planner**

    ---
    Any-angle grid planner with line-of-sight shortcuts.

    [:octicons-arrow-right-24: Go][theta-star-planner]

</div>

<span class="section-title">Controller Plugins</span>

<div class="grid cards bottom-align" markdown>

- :material-axis-arrow: **DWB Controller**

    ---
    Dynamic Window approach with configurable trajectory critics.

    [:octicons-arrow-right-24: Go][dwb-controller-index]

- :material-motion: **Graceful Controller**

    ---
    Smooth, graceful motion controller based on spirals.

    [:octicons-arrow-right-24: Go][graceful-controller]

- :material-chart-timeline-variant: **Model Predictive Path Integral Controller**

    ---
    Flagship sampling-based model predictive controller.

    [:octicons-arrow-right-24: Go][model-predictive-path-integral-controller]

- :material-target: **Regulated Pure Pursuit**

    ---
    Lookahead path follower with curvature and collision-based speed regulation.

    [:octicons-arrow-right-24: Go][regulated-pure-pursuit]

- :material-rotate-right: **Rotation Shim Controller**

    ---
    Rotate-in-place before handing off to the primary controller.

    [:octicons-arrow-right-24: Go][rotation-shim-controller]

</div>

<span class="section-title">Smoother Plugins</span>

<div class="grid cards bottom-align" markdown>

- :material-vector-curve: **Constrained Smoother**

    ---
    Optimization-based smoother respecting kinematic constraints.

    [:octicons-arrow-right-24: Go][constrained-smoother]

- :material-chart-bell-curve-cumulative: **Savitzky-Golay Smoother**

    ---
    Polynomial-fitting smoother for noise reduction.

    [:octicons-arrow-right-24: Go][savitzky-golay-smoother]

- :material-wave: **Simple Smoother**

    ---
    Lightweight iterative path smoother.

    [:octicons-arrow-right-24: Go][simple-smoother]

</div>

<span class="section-title">Others</span>

<div class="grid cards bottom-align" markdown>

- :material-crosshairs-gps: **AMCL**

    ---
    Adaptive Monte Carlo Localization for map-based pose estimation.

    [:octicons-arrow-right-24: Go][amcl]

- :material-view-grid: **Coverage Server**

    ---
    Complete coverage path planning for area-sweeping tasks.

    [:octicons-arrow-right-24: Go][coverage-server]

- :material-target-account: **Following Server**

    ---
    Dynamic object following with tracking recovery.

    [:octicons-arrow-right-24: Go][following-server]

- :material-replay: **Loopback Simulator**

    ---
    Simulated robot for testing without Gazebo.

    [:octicons-arrow-right-24: Go][loopback-simulator]

</div>
