# Environmental Representation { #environmental-representation }

The environmental representation is the way the robot perceives its environment.
It also acts as the central localization for various algorithms and data sources to combine their information into a single space.
This space is then used by the controllers, planners, and recoveries to compute their tasks safely and efficiently.

## Costmaps and Layers

The current environmental representation is a costmap.
A costmap is a regular 2D grid of cells containing a cost from unknown, free, occupied, or inflated cost.
This costmap is then searched to compute a global plan or sampled to compute local control efforts.

Various costmap layers are implemented as pluginlib plugins to buffer information into the costmap.
This includes information from LIDAR, RADAR, sonar, depth images, etc.
It may be wise to process sensor data before inputting it into the costmap layer, but that is up to the developer.

Costmap layers can be created to detect and track obstacles in the scene for collision avoidance using camera or depth sensors.
Additionally, layers can be created to algorithmically change the underlying costmap based on some rule or heuristic.
Finally, they may be used to buffer live data into the 2D or 3D world for binary obstacle marking.

## Costmap Filters { #environmental-representation-costmap-filters }

Imagine, you’re annotating a map file (or any image file) in order to have a specific action occur based on the location in the annotated map. Examples of marking/annotating might be keep out zones to avoid planning inside, or have pixels belong to maximum speeds in marked areas. This annotated map is called "filter mask". Just like a mask overlaid on a surface, it can or cannot be same size, pose and scale as a main map. The main goal of filter mask - is to provide the ability of marking areas on maps with some additional features or behavioral changes.

Costmap filters are a costmap layer-based approach of applying spatial-dependent behavioral changes, annotated in filter masks, into the Nav2 stack.
Costmap filters are implemented as costmap plugins.
These plugins are called "filters" as they are filtering a costmap by spatial annotations marked on filter masks.
In order to make a filtered costmap and change a robot’s behavior in annotated areas, the filter plugin reads the data coming from the filter mask.
This data is being linearly transformed into a feature map in a filter space.
Having this transformed feature map along with a map/costmap, any sensor data and current robot coordinate filters can update the underlying costmap and change the behavior of the robot depending on where it is.
For example, the following functionality could be made by use of costmap filters:

- Keep-out/safety zones where robots will never enter.
- Speed restriction areas. Maximum speed of robots going inside those areas will be limited.
- Preferred lanes for robots moving in industrial environments and warehouses.

## Other Forms

Various other forms of environmental representations exist.
These include:

- gradient maps, which are similar to costmaps but represent surface gradients to check traversibility over
- 3D costmaps, which represent the space in 3D, but then also requires 3D planning and collision checking
- Mesh maps, which are similar to gradient maps but with surface meshes at many angles
- "Vector space", taking in sensor information and using machine learning to detect individual items and locations to track rather than buffering discrete points.
