# ValidatePath { #validate-path }

Checks to see if the global path is valid. If there is an
obstacle along the path, it returns FAILURE, otherwise
it returns SUCCESS. Optionally checks specific costmap layers and
can use a custom footprint for validation.

{{ render_bt_node_ports(page.title) }}

## Example

```xml
<ValidatePath
  server_timeout="10"
  path="{path}"
  max_cost="100"
  consider_unknown_as_obstacle="false"
  layer_name=""
  footprint=""
  stop_at_first_collision="true"
  collision_poses="{collision_poses}" />
```

With max_lookahead_distance:

```xml
<ValidatePath
  path="{path}"
  max_lookahead_distance="5.0"
  collision_poses="{collision_poses}" />
```

With custom footprint:

```xml
<ValidatePath
  path="{path}"
  footprint="[[0.5,0.5],[0.5,-0.5],[-0.5,-0.5],[-0.5,0.5]]"
  collision_poses="{collision_poses}" />
```

Checking a specific costmap layer:

```xml
<ValidatePath
  path="{path}"
  layer_name="obstacle_layer"
  stop_at_first_collision="false"
  collision_poses="{collision_poses}" />
```
