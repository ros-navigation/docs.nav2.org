# Smoother Server { #smoother-server }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_smoother).

The Smoother Server implements the server for handling smooth path requests and hosting a vector of plugins implementing various C++ smoothers.
The server exposes an action interface for smoothing with multiple smoothers that share resources such as costmaps and TF buffers.

## Smoother Server Parameters

### **`costmap_topic`**

Type: `string` Default: `"global_costmap/costmap_raw"`

:   Raw costmap topic for collision checking.

### **`footprint_topic`**

Type: `string` Default: `"global_costmap/published_footprint"`

:   Topic for footprint in the costmap frame.

### **`transform_tolerance`**

Type: `double` Default: `0.1`

:   TF transform tolerance.

### **`action_server_result_timeout`**

Type: `double` Default: `10.0` Unit: `seconds`

:   The timeout value (in seconds) for action servers to discard a goal handle if a result has not been produced. This used to default to 15 minutes in rcl but was changed to 10 seconds in this [PR #1012](https://github.com/ros2/rcl/pull/1012), which may be less than some actions in Nav2 take to run.  For most applications, this should not need to be adjusted as long as the actions within the server do not exceed this deadline. This issue has been raised with OSRF to find another solution to avoid active goal timeouts for bookkeeping, so this is a semi-temporary workaround.

### **`robot_base_frame`**

Type: `string` Default: `"base_link"`

:   Robot base frame.

### **`smoother_plugins`**

Type: `vector<string>` Default: `{"nav2_smoother::SimpleSmoother"}`

:   List of plugin names to use, also matches action server names.

    Note
    :   Each plugin namespace defined in this list needs to have a `plugin` parameter defining the type of plugin to be loaded in the namespace.

        Example:
        ```yaml
        smoother_server:
          ros__parameters:
            smoother_plugins: ["simple_smoother", "curvature_smoother"]
            curvature_smoother:
              plugin: "nav2_ceres_costaware_smoother/CeresCostawareSmoother"
            simple_smoother:
              plugin: "nav2_smoother::SimpleSmoother"
        ```

### **`bond_heartbeat_period`**

Type: `double` Default: `0.1`

:   The lifecycle node bond mechanism publishing period (on the /bond topic). Disabled if inferior or equal to 0.0.

## Example

```yaml
smoother_server:
  ros__parameters:
    costmap_topic: global_costmap/costmap_raw
    footprint_topic: global_costmap/published_footprint
    robot_base_frame: base_link
    transform_tolerance: 0.1
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      do_refinement: True
```
