.. _galactic_migration:

Galactic to Humble
##################

Moving from ROS 2 Galactic to Humble, a number of stability improvements were added that we will not specifically address here.

Major improvements to Smac Planners
***********************************

The Smac Planner was significantly improved, of both the 2D and Hybrid-A* implementations, making the paths better, faster, and of higher quality.

  - Collision checker rejects collisions faster and queries the costmap for coordinates less often
  - Zero-copy collision checking object
  - precompute collision checking footprint orientations so no need for trig at runtime
  - Only checking full SE2 footprint when the robot is in the possibly inscribed zones
  - Computing the possibly inscribed zones, or the cost over which some part of the footprint may be in collision with a boundary to check the full footprint. Else, check center cost since promised to not be in a potential collision state
  - Renaming Hybrid-A* planner to SmacPlannerHybrid
  - Precomputing the Reedshepp and Dubin paths offline so at runtime its just a lookup table
  - Replacing the wavefront heuristic with a new, and novel, heuristic dubbed the obstacle heuristic. This computes a Dijkstra's path taking into account the 8 connected space, as well as weights for the cost at the positions to guide the heuristic into the center of aisle ways. It also downsamples the costmap such that it can reduce the number of expansions by 75% and have a very small error introduced into the heuristic by being off by at most a partial fraction of a single cell distance
  - Improvements to the analytic expansion algorithm to remove the possibility of loops at the end of paths, whenever possible to remove
  - 2D A* travel cost and heuristic improvements to speed up planning times and also increase the path quality significantly
  - Replaced smoother with a bespoke gradient descent implementation
  - Abstract out common utilities of planners into a utils file
  - tuned cost functions
  - precomputed obstacle heuristic using dynamic programming to expand only the minimum number of nodes
  - A caching heuristic setting to enable 25hz planning rates using cached obstacle heuristic values when the goal remains the same
  - Leveraging the symmetry in the dubin and reeds-sheep space to reduce cache size by 50% to increase the window size available for heuristic lookup.
  - Precompute primitives at all orientation bins

The tl;dr of these improvements is:
  - Plans are 2-3x as fast as they were before, well under 200ms for nearly all situations, making it as fast as NavFn and Global Planner (but now kinematically feasible). Typical planning times are sub-100ms without even making use of the caching or downsampling features.
  - Paths are of significantly higher quality via improved smoothers and a novel heuristic that steers the robot towards the center of aisleways implicitly. This makes smoother paths that are also further from obstacles whenever possible. 
  - Using caching or downsampler parameterizations, can easily achieve path planning with sub-50ms in nearly any sized space.