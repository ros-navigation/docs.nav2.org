<a id="bt-get-next-few-goals-action"></a>

# GetNextFewGoals

Extracts only the next `N` goals from a list of goals to send to a later task that only needs localized future knowledge.

## Input Ports

* **num_goals:**
  | Type   | Default   |
  |--------|-----------|
  | int    | N/A       |

  Description
  : How many of the goals to take from the input goals.
* **input_goals:**
  | Type           | Default   |
  |----------------|-----------|
  | nav_msgs/Goals | N/A       |

  Description
  : Input goals list.

## Output Ports

* **output_goals:**
  | Type           | Default   |
  |----------------|-----------|
  | nav_msgs/Goals | N/A       |

  Description
  : The output pruned goals list.

## Example

```xml
<GetNextFewGoals num_goals="3" input_goals="{goal_poses}" output_goals="{planning_goals}"/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
