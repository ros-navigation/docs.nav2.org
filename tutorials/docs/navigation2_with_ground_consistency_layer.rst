.. _navigation2_with_ground_consistency_layer:

Navigating with Ground Consistency Layer
****************************************

- `Overview`_
- `Practical Example`_
- `Ground Segmentation Overview`_
- `Ground Consistency Layer Mechanics`_
- `Configuration`_
- `Tuning Guide`_
- `Use Cases`_
- `Conclusion`_
- `Extended Topics`_

Overview
========

In this tutorial, we demonstrate a terrain-aware costmap layer for Nav2 which uses 3D lidar ground segmentation. The 
`ground consistency costmap layer <https://github.com/dfki-ric/nav2_ground_consistency_costmap_plugin>`_ allows users 
to classify terrain into traversable ground and obstacles for intelligent outdoor 
navigation in non-planar environments for smarter costmaps for safer navigation.

The Ground Consistency layer uses 
local terrain-relative obstacle and ground height reasoning based on ground segmentation rather than fixed global thresholds. This enables reliable 
navigation on slopes, under overhangs like bridges, through tunnels, and across uneven 
environments where traditional costmaps would struggle. 

.. image:: images/Navigation2_with_Ground_Consistency/ground_consistency_layer.gif
   :alt: Ground Consistency Layer Demo

Practical Example
=================

For a complete working example with Gazebo simulation, refer to the `Ground Consistency Demo <https://github.com/ros-navigation/navigation2_tutorials/tree/jazzy/nav2_ground_consistency_demo>`_. The demo includes:

- Pre-configured parameters for a Husky robot
- Example ground segmentation setup (KISS-ICP odometry)
- Simulation world with terrain for testing
- Complete launch files

The demo's README provides step-by-step instructions for running the full system.

Ground Segmentation Overview
============================

The Ground Consistency layer operates on point clouds that are pre-segmented into ground and non-ground points. 
This segmentation is performed by a ground segmentation algorithm, which processes sensor data 
in real time as the robot moves and determines whether each point belongs to the terrain surface or lies 
above it as a potential obstacle. Without the segmentation, 
the layer cannot distinguish between true obstacles and normal variations like slopes or uneven ground. 

There are many well-established ground segmentation methods; in this tutorial, we use the 
`ground_segmentation_ros2 <https://github.com/dfki-ric/ground_segmentation_ros2>`_
package developed by DFKI Robotics Innovation Center.

Ground Consistency Layer Mechanics
==================================

The Ground Consistency layer implements an evidence-based probabilistic approach for cell occupancy 
estimation by maintaining accumulated evidence rather than making binary decisions on individual 
sensor observations. This section describes the three core mechanisms.

1. Evidence Accumulation and Competition System
-----------------------------------------------

Each grid cell collects two types of evidence: ground and obstacle. As new sensor data arrives, these 
scores are updated and compared to estimate how likely the cell is occupied.

A cell is marked as an obstacle only when there is both:

- enough evidence, and
- high confidence that the evidence indicates an obstacle

This prevents isolated sensor noise from affecting navigation. Cells transition gradually between free and 
occupied states as evidence builds or fades.

2. Height-Based Occupancy Classification
----------------------------------------

Not all detected obstacles actually block the robot. The layer evaluates obstacle height relative to the 
local ground height. Based on the robot's height:

- Very high objects are treated as overhead (safe to pass under)
- Very low objects are treated as terrain variation
- Only objects within the robot’s collision range are considered blocking

At times, the terrain is such that no local ground height can be reliably determined. In this case, the 
layer can be configured to use neighbour cells to estimate ground height, or treat all such obstacles 
without ground below them as blocking.

3. Temporal Stability Through Evidence Decay
--------------------------------------------

The evidences are decayed over time to allow the costmap to adapt to changing environments. The amount of 
decay can be tuned separately for ground and obstacle evidence, creating temporal hysteresis.

Configuration
==============

Now we configure Nav2 to use the ground consistency layer in its costmaps. We add the layer plugin 
to the global and local costmaps and configure the parameters based on our robot and environment.

Let us look at an example configuration for the local costmap:

.. code-block:: yaml

   local_costmap:
     local_costmap:
       ros__parameters:
         # ... other costmap settings ...
         plugins: ["ground_consistency", "other_plugins..."]
         
         ground_consistency:
           plugin: "nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer"
           ground_points_topic: /ground_segmentation/ground_points
           nonground_points_topic: /ground_segmentation/obstacle_points
           tf_timeout: 0.1
           min_clearance: 0.1
           robot_height: 0.92
           maximum_height_filter: 2.0
           ground_inc: 1.0
           nonground_inc: 1.5
           nonground_decay: 0.93
           ground_decay: 0.80
           max_score: 5000.0
           nonground_occ_thresh: 6.0
           nonground_prob_thresh: 0.75
           enable_kpi_logging: false
           discretize_costs: true
           max_data_range: 50.0
           ground_neighbor_search_cells: 0

Parameters Reference
--------------------
.. list-table::
   :header-rows: 1

   * - Name
     - Type
     - Default
     - Description
   * - ``ground_points_topic``
     - string
     - /ground_points
     - Topic providing ground points (PointCloud2)
   * - ``nonground_points_topic``
     - string
     - /nonground_points
     - Topic providing obstacle points (PointCloud2)
   * - ``robot_height``
     - double
     - 1.2
     - Robot height in meters (used for height filtering)
   * - ``tf_timeout``
     - double
     - 0.1
     - Timeout for TF lookups (seconds)
   * - ``ground_inc``
     - double
     - 1.0
     - Evidence added per ground point
   * - ``nonground_inc``
     - double
     - 1.5
     - Evidence added per obstacle point (typically higher than ``ground_inc``)
   * - ``max_score``
     - double
     - 5000.0
     - Maximum accumulated evidence per cell
   * - ``ground_decay``
     - double
     - 0.80
     - Ground evidence decay factor (faster forgetting)
   * - ``nonground_decay``
     - double
     - 0.93
     - Obstacle evidence decay factor (slower forgetting)
   * - ``min_clearance``
     - double
     - 0.1
     - Minimum height to consider an obstacle blocking
   * - ``maximum_height_filter``
     - double
     - 2.0
     - Non-ground points above this height are ignored. The threshold is applied in the coordinate frame of the topic ``/nonground_points``, not the global frame.   
   * - ``ground_neighbor_search_cells``
     - int
     - 0
     - Neighbor radius for ground height estimation (0 = disabled)
   * - ``nonground_occ_thresh``
     - double
     - 6.0
     - Minimum obstacle evidence required before occupancy is considered
   * - ``nonground_prob_thresh``
     - double
     - 0.75
     - Probability threshold for marking a cell as occupied
   * - ``discretize_costs``
     - bool
     - true
     - Convert cost values to binary (LETHAL or FREE) instead of continuous costs  
   * - ``max_data_range``
     - double
     - 50.0
     - Maximum sensor range used. All cells beyond this distance are erased to prevent stale data from far away.
   * - ``enable_kpi_logging``
     - bool
     - false
     - Enable KPI logging of layer performance metrics (for development/debugging)  
     
Tuning Guide
============

The layer's behavior depends on your specific use case (terrain type, robot size, sensor characteristics). Here's how to tune key parameters:
     
**Robot Height** (``robot_height``)
   - Set to your robot's actual height
   - The layer uses this to classify obstacles as blocking or non-blocking based on their height relative to the ground.
   - Example: Husky robot = 0.92m

**Evidence Accumulation** (``ground_inc``, ``nonground_inc``)
   - Higher increments = faster response to observations
   - Keep ``nonground_inc > ground_inc`` because we want nonground evidence to accumulate faster than ground evidence. This creates a bias towards safety.
   - Typical ratio: 1.0 ground, 1.5 nonground

**Evidence Decay** (``ground_decay``, ``nonground_decay``)
   - **Ground decay** (lower value): We decay ground evidence faster to allow the costmap to adapt quickly to changes in terrain (e.g., moving onto a slope). 
     Use 0.80-0.85 for responsive terrain adaptation.
   - **Nonground decay** (higher value): We want obstacles to persist longer to prevent flickering due to sensor noise. Use 0.90-0.95 for stable obstacle marking.
   - Difference creates temporal hysteresis: obstacles are believed longer than ground evidence.

**Height Filtering** (``min_clearance``, ``maximum_height_filter``)
   - ``min_clearance``: Minimum bump your robot can detect. Too low = sensitive to noise; too high = misses small obstacles.
   - ``maximum_height_filter``: Overhead canopy/ceiling height. Objects above this are ignored.
   - At times, ground segmentation may classify ground points as non-ground (e.g., due to sensor noise or very uneven terrain). 
     Setting a reasonable ``min_clearance`` can prevent these misclassifications from blocking navigation.

**Neighbor Search** (``ground_neighbor_search_cells``)
   - 0 = Use only ground points in current cell for height estimation
   - 1-3 = Average ground height from neighboring cells (more stable but slower response)
   - Use on very uneven terrain where cells might lack ground points or when low resolution lidar data causes sparse ground points.

**Decision Thresholds** (``nonground_occ_thresh``, ``nonground_prob_thresh``)
   These two thresholds work together in a two-stage filter:
   
   - ``nonground_occ_thresh``: **Minimum score (accumulated evidence) required**. With default ``nonground_inc: 1.5``, a value of 6.0 means you need ~4 obstacle point observations before considering a cell for LETHAL marking. This is the primary defense against stray sensor noise.
   - ``nonground_prob_thresh``: **Probability confidence required** (after score passes). Even with enough accumulated evidence, the observations must agree with high confidence (default 75%) to actually mark as LETHAL.
   
   **Why both?** Alone, ``nonground_prob_thresh`` would fire on a single high-confidence false positive point. Together, they require **both**: sustained evidence (multiple observations) **and** high agreement (high confidence). This combination prevents isolated false positives from blocking navigation.
   
   - **For aggressive navigation** (narrow spaces): Decrease ``nonground_occ_thresh`` to 4-5, increase ``nonground_prob_thresh`` to 0.85 (requires stronger evidence per point)
   - **For conservative navigation** (safety-critical): Increase ``nonground_occ_thresh`` to 8-10, decrease ``nonground_prob_thresh`` to 0.5 (generous with evidence accumulation)

Use Cases
=========

**Outdoor Navigation on Uneven Terrain**
   - **Problem**: Standard costmaps mark slopes as obstacles
   - **Solution**: Ground consistency ignores traversable slopes while detecting real blocking obstacles
   - **Tuning**: Lower decay values to respond quickly to terrain changes

**Navigation Under Structures (Bridges, Overpasses)**
   - **Problem**: Standard costmaps block navigation under overhead structures
   - **Solution**: Layer marks overhead structures as non-blocking
   - **Tuning**: Set ``maximum_height_filter`` to the maximum obstacle height you want the layer to evaluate; objects above this are treated as overhead/non-blocking.

**Forest/Cluttered Navigation**
   - **Problem**: Branches, leaves, and small debris create false obstacles
   - **Solution**: Only actual blocking-height obstacles are marked
   - **Tuning**: Adjust ``min_clearance`` to filter out debris below robot height

**Temporal Stability in Dynamic Environments**
   - **Problem**: Sensor noise creates flickering obstacles
   - **Solution**: Evidence accumulation and decay smooth the costmap over time
   - **Tuning**: Increase decay values to smooth out noise, decrease to respond faster to changes

Conclusion
==========

The Ground Consistency costmap layer enables terrain-aware navigation by:

1. ✅ **Understanding terrain geometry** - Distinguishes traversable slopes from blocking obstacles
2. ✅ **Height-aware filtering** - Marks only obstacles that actually block the robot
3. ✅ **Temporal smoothing** - Uses evidence accumulation to create stable, responsive costmaps
4. ✅ **Flexible integration** - Works with any ground segmentation algorithm

You can integrate the layer into any Nav2-based robot by:

1. Installing the plugin
2. Providing ground/non-ground point clouds (from your choice of sensor/algorithm)
3. Configuring the parameters for your robot dimensions and terrain
4. Adding it to your costmap configuration

The layer's behavior is highly tunable - start with the provided defaults, then adjust based on your specific terrain and robot characteristics.

Extended Topics
===============

Troubleshooting
---------------

**Layer not being used in costmap**
   - Verify the plugin is installed: ``ros2 pkg list | grep ground_consistency``
   - Check that the layer name matches in your configuration (``ground_consistency``)
   - Ensure the plugin name is fully qualified: ``nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer``

**No cells marked as obstacles**
   - Verify ground and non-ground point topics are being published
   - Check that point cloud data is arriving: ``ros2 topic hz /ground_segmentation/obstacle_points``
   - If no points arrive, the segmentation algorithm may not be running or publishing to wrong topics
   - Verify ``nonground_occ_thresh`` isn't too high

**Costmap too conservative (marks too many obstacles)**
   - Decrease ``nonground_inc`` (evidence accumulates slower)
   - Increase ``ground_decay`` (ground evidence fades faster)
   - Increase ``nonground_occ_thresh`` (higher evidence needed to mark as occupied)
   - Verify ``robot_height`` is correct

**Costmap too aggressive (misses obstacles)**
   - Increase ``nonground_inc`` (evidence accumulates faster)
   - Decrease ``nonground_decay`` (obstacles persist longer)
   - Decrease ``nonground_occ_thresh`` (lower evidence to mark as occupied)
   - Verify point cloud data quality from segmentation algorithm

**Overhead structures blocking navigation**
   - Increase ``maximum_height_filter`` to the height of your structures
   - Verify that overhead points are actually coming through in the point cloud
   - Check that ``robot_height`` is correctly set

**Performance Issues**
   - Reduce ``max_data_range`` if using only nearby points
   - Disable ``enable_kpi_logging`` in production
   - Verify that ``ground_neighbor_search_cells`` is set appropriately (0 is fastest)

Ground Segmentation Sources
---------------------------

The Ground Consistency layer requires two input streams: ground and non-ground points. The quality of these classifications directly affects layer performance.

**Recommended approaches:**

- **3D Lidar-based**: Fast, works in varying light. Examples: VLP-16, Ouster, Sick
- **Camera-based**: RGB-D or stereo. Works indoors, can be affected by lighting
- **Multi-sensor fusion**: Combine multiple sensors for robustness

**Important considerations:**

- Points must be in the costmap frame (usually ``base_link``)
- Publish rates should match your costmap update frequency
- Point cloud quality is more important than quantity
- Ground points should represent actual terrain surface
- Non-ground points should represent true obstacles only

Funding
=======

Developed at the Robotics Innovation Center (DFKI), Bremen. Supported by Robdekon2 (50RA1406), German Federal Ministry for Research and Technology.
