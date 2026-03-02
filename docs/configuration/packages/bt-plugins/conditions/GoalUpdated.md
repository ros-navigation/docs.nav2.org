<a id="goal-updated-condition"></a>

# GoalUpdated

Checks if the global navigation goal, or a vector of goals, has changed in the blackboard.
Returns failure if the goal is the same, if it changes, it returns success.

## Input Ports

* **goal:**
  | Type                            | Default   |
  |---------------------------------|-----------|
  | geometry_msgs::msg::PoseStamped | “{goal}”  |

  Description
  : Destination to check. Takes in a blackboard variable, “{goal}” if not specified.
* **goals:**
  | Type                 | Default   |
  |----------------------|-----------|
  | nav_msgs::msg::Goals | “{goals}” |

  Description
  : Vector of goals to check. Takes in a blackboard variable, “{goals}” if not specified.

## Example

```xml
<GoalUpdated/>
```

<!-- These are replacement strings for non-ASCII characters used within the project
using the same name as the html entity names (e.g., &copy;) for that character -->
