# Coverage Server { #coverage-server }

Source code on [Github](https://github.com/open-navigation/opennav_coverage).

The Coverage Server in `opennav_coverage` implements the server for handling the complete-coverage planning requests of a given field or zone set in cartesian or GPS coordinates using the Fields2Cover library.
It can also compute coverage route and plans using a precomputed set of rows using the `opennav_row_coverage` server.
It is within the `opennav_coverage` project, not within Nav2 directly, but is planned for a longer-term integration once a few key features are available in Fields2Cover. If you wish to contribute to this effort, please let a maintainer know!

!!! note

    All `default_` prefixed parameters can be overwritten in the action request field. When modes are not set in the Action goal, the defaults are utilized.

## Parameters

### **`coordinates_in_cartesian_frame`**

Type: `bool` Default: `true`

:   Whether or not requests coming into the server will be in cartesian (e.g. meters) or GPS coordinates. If GPS, they are automatically converted into UTM frame (for meters) to compute the coverage paths, then converted back into GPS for the client.

### **`robot_width`**

Type: `double` Default: `2.1`

:   The robot's width in meters.

### **`operation_width`**

Type: `double` Default: `2.5`

:   The robot's operational width (cleaning, planting, etc) for computing coverage swath distances

### **`min_turning_radius`**

Type: `double` Default: `0.4`

:   The robot's minimum turning radius for computing paths connecting route swaths (m)

### **`linear_curv_change`**

Type: `double` Default: `2.0`

:   The robot's maximum linear curvature change for computing paths connecting route swaths (1/m^2)

### **`default_allow_overlap`**

Type: `bool` Default: `false`

:   Whether, by default, to allow overlapping of the last row in the coverage plan to obtain coverage at the far edge. Only for `opennav_coverage`.

### **`default_headland_type`**

Type: `string` Default: `"CONSTANT"`

:   The default headland generation method. Constant is the only valid method currently. Only for `opennav_coverage`.

### **`default_headland_width`**

Type: `double` Default: `2.0`

:   The default headland width to remove from the field or zone from coverage planning. Only for `opennav_coverage`.

### **`default_swath_type`**

Type: `string` Default: `"LENGTH"`

:   Objective to use to score swath generation candidates at different angles when using `"BRUTE_FORCE"` swath angle type. Options: `"LENGTH"`, `"COVERAGE"`, `"NUMBER"` for `opennav_coverage`. Option: `"OFFSET"`, `"CENTER"`, `"ROWSARESWATHS"` for `opennav_row_coverage`.
    Note that `"COVERAGE"` takes 10x longer than others.

### **`default_swath_angle_type`**

Type: `string` Default: `"BRUTE_FORCE"`

:   Mode to use for generating swaths. Need to find optimal angle by the swath generator objectives, if not given. Options: `"BRUTE_FORCE"`, `"SET_ANGLE"`. Only for `opennav_coverage`.

### **`default_step_angle`**

Type: `double` Default: `1.7e-2`

:   The angular step size to try to find the optimal angle for route objective, when using `"BRUTE_FORCE"` swath angle type. Default is 1 deg in rad units. Only for `opennav_coverage`.

### **`default_swath_angle`**

Type: `double` Default: `N/A`

:   The optimal angle for route objective, when using `"SET_ANGLE"` swath angle type. Default is 1 deg in rad units. Only for `opennav_coverage`.

### **`default_route_type`**

Type: `string` Default: `"BOUSTROPHEDON"`

:   Default order when computing routes to order swaths. Options: `"BOUSTROPHEDON"`, `"SNAKE"`, `"SPIRAL"`, `"CUSTOM"`

### **`default_custom_order`**

Type: `vector<int>` Default: `N/A`

:   The default custom swath order for the route planner in the `"CUSTOM"` mode. The length of this custom order must be `>= swaths.size()`. Only relevant when using the `"CUSTOM"` Route Type.

### **`default_spiral_n`**

Type: `int` Default: `4`

:   Default number of swaths to skip and double back on to create a spiral pattern in the route. Only relevant when using the `"SPIRAL"` Route Type. `"SNAKE"` is a special case when Spiral N = 2.

### **`default_path_continuity_type`**

Type: `string` Default: `"CONTINUOUS"`

:   Default continuity type when computing paths to connect routes together. Options `"DISCONTINUOUS"`, `"CONTINUOUS"`.

### **`default_path_type`**

Type: `string` Default: `"DUBIN"`

:   Default type when computing paths to connect routes together using curves. Options: `"DUBIN"`, `"REEDS_SHEPP"`.

### **`default_turn_point_distance`**

Type: `double` Default: `0.1`

:   Distance between points on the plan and route for sending back in paths (e.g. 0.1m). This impacts the density of the output turn paths and the overall nav paths.

### **`default_offset`**

Type: `double` Default: `0.0`

:   Offset to use for computing swaths from annotated rows. Only for `opennav_row_coverage`.

### **`order_ids`**

Type: `bool` Default: `false`

:   For `opennav_row_coverage`, whether to reorder the parsed rows in the order of their `id` s.

## Example

```yaml
coverage_server:
  ros__parameters:
    coordinates_in_cartesian_frame: true
    robot_width: 2.1
    operation_width: 2.5
    min_turning_radius: 0.4
    linear_curv_change: 2.0
    default_allow_overlap: true
    default_headland_width: 0.5
    default_path_continuity_type: "CONTINUOUS"
    default_path_type: "DUBINS"
    default_route_type: "BOUSTROPHEDON"
    default_swath_angle_type: "BRUTE_FORCE"
    default_swath_type: "LENGTH"
    default_turn_point_distance: 0.1
```
