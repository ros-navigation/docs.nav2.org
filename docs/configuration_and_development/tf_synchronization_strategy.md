# Transform Synchronization Strategy { #transform-synchronization-strategy }

Nav2 combines sensor observations, robot poses, paths, and commands that may have been produced at different times. A transform that is valid now may not describe where the robot or sensor was when older data was recorded. Preserve timestamps when moving data between frames so each operation uses a consistent view of the robot and its surroundings.

## Use the timestamp of the data

| Data or operation | Time to use | Transform manager API |
| --- | --- | --- |
| Sensor observations, such as scans and point clouds | The observation's capture timestamp | `getStampedTransform(target_frame, source_frame, stamp, transform_tolerance)`, then `tf2::doTransform` for supported message types |
| Stamped computed results, such as paths and poses | The result's own timestamp | `transformStampedPose(target_frame, pose, transform_tolerance)` for poses; `getStampedTransform()` and `tf2::doTransform` for other data |
| The robot's current pose or another current frame relationship | The latest common time available along the TF chain | `getLatestPose()` or `getLatestTransform()`, with a positive staleness threshold when old TF must be rejected |
| A source observation at one time expressed in a target frame at another time | Both timestamps, through a fixed frame such as `odom` | `getStampedTransform(target_frame, target_time, source_frame, source_time, fixed_frame, transform_tolerance)` |

When a server needs a current robot pose, obtain it once for that operation and carry its timestamp through downstream calculations. If several values must refer to the same instant, reuse that timestamp or the resolved transform rather than independently requesting the latest transform for each value. Preserve the original timestamp of sensor data and computed results unless an operation intentionally changes the time being represented.

Use `nav2::TransformManager` from `nav2_ros_common/tf_manager.hpp` for these lookups and for `sendTransform()` or `sendStaticTransform()` when broadcasting. See the [Lyrical to M-Turtle migration guide][lyrical-to-m-turtle] for the API mapping and staleness parameter.
