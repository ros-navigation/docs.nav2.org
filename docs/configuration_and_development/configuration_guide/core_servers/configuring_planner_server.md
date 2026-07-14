# Planner Server { #planner-server }

Source code on [Github](https://github.com/ros-navigation/navigation2/tree/main/nav2_planner).

The Planner Server implements the server for handling the planner requests for the stack and host a map of plugin implementations.
It will take in a goal and a planner plugin name to use and call the appropriate plugin to compute a path to the goal.
It also hosts the global costmap.

## Parameters

### **`planner_plugins`**

Type: `vector<string>` Default: `['GridBased']`

:   List of Mapped plugin names for parameters and processing requests.

    Note
    :   Each plugin namespace defined in this list needs to have a `plugin` parameter defining the type of plugin to be loaded in the namespace.

        Example:
        ```yaml
        planner_server:
          ros__parameters:
            planner_plugins: ["GridBased"]
            GridBased:
              plugin: "nav2_navfn_planner::NavfnPlanner"
        ```

### **`allow_partial_planning`**

Type: `bool` Default: `false`

:   Allows planner server to output partial paths in the presence of obstacles when planning through poses. Otherwise planner fails and aborts the plan request in such a case by default.

### **`expected_planner_frequency`**

Type: `double` Default: `1.0`

:   Expected planner frequency. If the current frequency is less than the expected frequency, display the warning message.

### **`bond_heartbeat_period`**

Type: `double` Default: `0.25`

:   The lifecycle node bond mechanism publishing period (on the /bond topic). Disabled if inferior or equal to 0.0.

### **`costmap_update_timeout`**

Type: `double` Default: `1.0`

:   The timeout value (seconds) for the costmap to be fully updated before a planning request.

### **`introspection_mode`**

Type: `string` Default: `"disabled"`

:   The introspection mode for services and actions. Options are "disabled", "metadata", "contents".

### **`allow_parameter_qos_overrides`**

Type: `bool` Default: `true`

:   Whether to allow QoS profiles to be overwritten with parameterized values.

## Default Plugins

When the `planner_plugins` parameter is not overridden, the following default plugins are loaded:

| Namespace   | Plugin                             |
|-------------|------------------------------------|
| "GridBased" | "nav2_navfn_planner::NavfnPlanner" |

## Example

```yaml
planner_server:
  ros__parameters:
    allow_partial_planning: false
    expected_planner_frequency: 20.0
    costmap_update_timeout: 1.0
    introspection_mode: "disabled"
    planner_plugins: ['GridBased']
    GridBased:
      plugin: 'nav2_navfn_planner::NavfnPlanner'
```
